#include "easyfly_mission_fsm/mission_fsm.hpp"
#include "easyfly_mission_fsm/mission_format.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
using namespace std::chrono_literals;
namespace easyfly_mission_fsm {
using Clock = std::chrono::steady_clock;
using Status = easyfly_core::Status;
using MS = easyfly_msgs::msg::MissionStatus;
static std::string backend(const std::string &type) {
  return type == "waypoint" ? "goto" : type == "hover" ? "" : type;
}
static uint8_t phase(const std::string &type) {
  if(type=="takeoff") return MS::TAKEOFF;
  if(type=="hover") return MS::HOVER;
  if(type=="track") return MS::TRACK;
  if(type=="search" || type=="explore") return MS::SEARCH;
  if(type=="land" || type=="precision_land") return MS::LAND;
  return MS::WAYPOINT;
}
MissionFSM::MissionFSM(const rclcpp::NodeOptions &options):Node("mission_fsm",options) {
  easyfly_core::ClientOptions sdk_options;
  sdk_options.use_sim_time = get_parameter("use_sim_time").as_bool();
  sdk_options.require_managed_ownership = true;
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only=true;
  sdk_options.state_freshness_seconds = declare_parameter("state_freshness_s",1.0,read_only);
  sdk_ = std::make_unique<easyfly_core::Client>(sdk_options);
  cancel_budget_s_ = declare_parameter("cancel_budget_s",3.0,read_only);
  battery_invalid_grace_s_ = declare_parameter("battery_invalid_grace_s",5.0,read_only);
  backend_discovery_s_ = declare_parameter("backend_discovery_s",3.0,read_only);
  if(sdk_options.state_freshness_seconds<.05 || sdk_options.state_freshness_seconds>10 ||
     !std::isfinite(cancel_budget_s_) || cancel_budget_s_<.05 || cancel_budget_s_>30 ||
     !std::isfinite(battery_invalid_grace_s_) || battery_invalid_grace_s_<0 || battery_invalid_grace_s_>60 ||
     !std::isfinite(backend_discovery_s_) || backend_discovery_s_<0 || backend_discovery_s_>30)
    throw std::invalid_argument("invalid mission timing parameters");
  pub_status_=create_publisher<MS>("/mission_status",10);
  pub_step_feedback_=create_publisher<easyfly_msgs::msg::MissionStepFeedback>("/mission/step_feedback",1);
  cli_set_mode_=create_client<easyfly_msgs::srv::SetFlightMode>("/set_flight_mode");
  as_mission_=rclcpp_action::create_server<ExecuteMission>(this,"/execute_mission",
    std::bind(&MissionFSM::handle_mission_goal,this,std::placeholders::_1,std::placeholders::_2),
    std::bind(&MissionFSM::handle_mission_cancel,this,std::placeholders::_1),
    std::bind(&MissionFSM::handle_mission_accepted,this,std::placeholders::_1));
  ownership_service_=create_service<easyfly_msgs::srv::GetActionOwnership>("/mission/ownership",
    [this](easyfly_msgs::srv::GetActionOwnership::Request::SharedPtr request,
           easyfly_msgs::srv::GetActionOwnership::Response::SharedPtr response) {
      response->goal_id=request->goal_id;
      response->state=ownership_registry_.state(request->goal_id);
    });
  timer_safety_=create_wall_timer(100ms,std::bind(&MissionFSM::safety_check,this));
  publish_status(MS::IDLE,"idle",-1,0,0);
}
MissionFSM::~MissionFSM() {
  shutdown_=true; stop("shutdown"); cancel_current_step();
  std::lock_guard<std::mutex> lock(worker_mtx_);
  if(worker_.joinable()) worker_.join();
  sdk_->close(0); // unresolved remote ownership remains in SDK's self-owned drain
}
bool MissionFSM::state_ready(const easyfly_core::StateSnapshot &s) const {
  const auto &u=s.value;
  return s.fresh && u.connected && u.pose_valid && u.battery_valid &&
    u.flight_mode!=6 && u.flight_mode!=7 &&
    std::isfinite(u.position_x) && std::isfinite(u.position_y) && std::isfinite(u.position_z) &&
    std::isfinite(u.battery_remaining) && u.battery_remaining>=0 && u.battery_remaining<=1;
}
void MissionFSM::stop(const std::string &why) {
  std::lock_guard<std::mutex> lock(mission_mtx_);
  if(stop_reason_.empty()) stop_reason_=why;
}
std::string MissionFSM::reason() const {
  std::lock_guard<std::mutex> lock(mission_mtx_); return stop_reason_;
}
void MissionFSM::cancel_current_step() {
  std::lock_guard<std::mutex> lock(step_mtx_);
  if(current_) current_->cancel();
}
void MissionFSM::trigger_rtl() {
  if(cli_set_mode_->service_is_ready()) {
    auto request=std::make_shared<easyfly_msgs::srv::SetFlightMode::Request>();
    request->flight_mode=6; cli_set_mode_->async_send_request(request);
  } else RCLCPP_ERROR(get_logger(),"low-battery RTL service unavailable");
}
void MissionFSM::safety_check() {
  if(!mission_active_) {battery_invalid_since_={}; return;}
  const auto snapshot=sdk_->state(); const auto &u=snapshot.value;
  if(!snapshot.fresh || !u.connected || !u.pose_valid ||
     !std::isfinite(u.position_x) || !std::isfinite(u.position_y) || !std::isfinite(u.position_z)) {
    stop("state_invalid_or_stale"); cancel_current_step(); return;
  }
  // LAND is expected only while executing our own land action.
  bool owned_land=false;
  { std::lock_guard<std::mutex> lock(step_mtx_);
    owned_land=current_type_=="land" || current_type_=="precision_land"; }
  if(u.flight_mode==6) {stop("external_rtl_takeover"); cancel_current_step();}
  if(u.flight_mode==7 && !owned_land) {stop("external_land_takeover"); cancel_current_step();}
  if(!u.battery_valid || !std::isfinite(u.battery_remaining) || u.battery_remaining<0 || u.battery_remaining>1) {
    if(battery_invalid_since_==Clock::time_point{}) battery_invalid_since_=Clock::now();
    if(std::chrono::duration<double>(Clock::now()-battery_invalid_since_).count()>=battery_invalid_grace_s_ &&
       !rtl_triggered_.exchange(true)) {
      stop("battery_invalid_timeout");
      if(u.armed && u.position_z>.3) trigger_rtl();
      cancel_current_step();
    }
  } else {
    battery_invalid_since_={};
    if(u.battery_remaining<.15 && !rtl_triggered_.exchange(true)) {
      stop("low_battery");
      if(u.armed && u.position_z>.3) trigger_rtl();
      cancel_current_step();
    }
  }
}
rclcpp_action::GoalResponse MissionFSM::handle_mission_goal(
  const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const ExecuteMission::Goal> goal) {
  const auto identity=easyfly_core::goalIdentity(uuid);
  if(mission_active_ || shutdown_) {
    RCLCPP_WARN(get_logger(),"mission_admission reject=busy_or_shutdown goal_uuid=%s",identity.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto snapshot=sdk_->state();
  if(!state_ready(snapshot)) {
    const auto &u=snapshot.value;
    RCLCPP_WARN(get_logger(),
      "mission_admission reject=state_not_ready goal_uuid=%s received=%d fresh=%d receipt_fresh=%d "
      "source_time_valid=%d receipt_age=%.9f source_age=%.9f connected=%d pose_valid=%d "
      "battery_valid=%d battery=%.9f mode=%u position_finite=%d",
      identity.c_str(),snapshot.received,snapshot.fresh,snapshot.receipt_fresh,snapshot.source_time_valid,
      snapshot.receipt_age_seconds,snapshot.source_age_seconds,u.connected,u.pose_valid,
      u.battery_valid,static_cast<double>(u.battery_remaining),static_cast<unsigned>(u.flight_mode),
      std::isfinite(u.position_x) && std::isfinite(u.position_y) && std::isfinite(u.position_z));
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto validation=validateMissionFile(goal->mission_file);
  if(!validation.valid()) {
    RCLCPP_WARN(get_logger(),"mission_admission reject=invalid_mission_file goal_uuid=%s",identity.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::lock_guard<std::mutex> lock(mission_mtx_);
  if(mission_active_.exchange(true)) {
    RCLCPP_WARN(get_logger(),"mission_admission reject=concurrent_goal goal_uuid=%s",identity.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  accepted_steps_=validation.steps; stop_reason_.clear();
  cancel_requested_=false; rtl_triggered_=false; steps_completed_=0;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse MissionFSM::handle_mission_cancel(std::shared_ptr<Handle>) {
  cancel_requested_=true; stop("user_cancel"); cancel_current_step();
  return rclcpp_action::CancelResponse::ACCEPT;
}
void MissionFSM::handle_mission_accepted(std::shared_ptr<Handle> handle) {
  ownership_registry_.own(easyfly_core::goalIdentity(handle->get_goal_id()));
  std::lock_guard<std::mutex> worker_lock(worker_mtx_);
  if(worker_.joinable()) worker_.join();
  std::vector<MissionStep> steps;
  {std::lock_guard<std::mutex> lock(mission_mtx_); steps=accepted_steps_;}
  worker_=std::thread([this,handle,steps] {execute_mission(steps,handle);});
}
void MissionFSM::publish_status(uint8_t p,const std::string &name,int index,int total,float progress,
  bool success,const std::string &message) {
  MS s; s.header.stamp=now(); s.mission_phase=p; s.phase_name=name;
  s.current_waypoint=index; s.total_waypoints=total; s.progress=progress;
  s.mission_success=success; s.message=message; pub_status_->publish(s);
}
void MissionFSM::feedback(uint8_t p,const std::string &name,int index,int total,float progress,
  std::shared_ptr<Handle> handle) {
  auto f=std::make_shared<ExecuteMission::Feedback>(); auto &s=f->current_status;
  s.header.stamp=now(); s.mission_phase=p; s.phase_name=name;
  s.current_waypoint=index; s.total_waypoints=total; s.progress=progress;
  s.message=reason(); handle->publish_feedback(f);
  publish_status(p,name,index,total,progress,false,s.message);
}
void MissionFSM::execute_mission(std::vector<MissionStep> steps,std::shared_ptr<Handle> handle) {
  const int total=static_cast<int>(steps.size());
  try {
    // Discover every required backend before the first flight action. No sync
    // client wait occurs on the ROS executor thread.
    std::set<std::string> required;
    for(const auto &s:steps) if(!backend(s.type).empty()) required.insert(backend(s.type));
    const auto deadline=Clock::now()+std::chrono::duration<double>(backend_discovery_s_);
    while(reason().empty() && !shutdown_) {
      bool ready=true; for(const auto &name:required) ready &= sdk_->server_ready(name);
      if(ready) break;
      if(Clock::now()>=deadline) {stop("required_backend_unavailable"); break;}
      std::this_thread::sleep_for(20ms);
    }
    for(int i=0;i<total && reason().empty() && !shutdown_;++i)
      if(!execute_step(steps[i],i,total,handle)) break;
  } catch(const std::exception &e) {
    stop(std::string("internal_error:")+e.what()); cancel_current_step();
    std::shared_ptr<easyfly_core::Operation> operation;
    {std::lock_guard<std::mutex> lock(step_mtx_); operation=current_;}
    const auto deadline=Clock::now()+std::chrono::duration<double>(cancel_budget_s_);
    while(operation && !shutdown_ && operation->snapshot().remote_execution_pending && Clock::now()<deadline)
      operation->wait(.02);
  }
  auto result=std::make_shared<ExecuteMission::Result>();
  result->success=steps_completed_==total && reason().empty() && !shutdown_;
  result->message=result->success ? "mission complete" : reason().empty() ? "step failed" : reason();
  try {
    if(cancel_requested_ && handle->is_canceling()) handle->canceled(result);
    else if(result->success) handle->succeed(result); else handle->abort(result);
  } catch(const std::exception &e) {RCLCPP_WARN(get_logger(),"public result: %s",e.what());}
  // A public failure/cancel cannot release an unknown live child. Keep the
  // mission slot fault-locked until a real remote terminal arrives.
  std::shared_ptr<easyfly_core::Operation> operation;
  {std::lock_guard<std::mutex> lock(step_mtx_); operation=current_;}
  while(operation && operation->snapshot().remote_execution_pending && !shutdown_) {
    try {publish_status(MS::EMERGENCY,"fault_owned_child",steps_completed_,total,
      static_cast<float>(steps_completed_)/total,false,result->message);} catch(const std::exception &) {}
    operation->cancel(); operation->wait(.05);
  }
  {std::lock_guard<std::mutex> lock(step_mtx_); current_.reset();}
  try {publish_status(MS::IDLE,result->success?"complete":"idle",steps_completed_,total,
    static_cast<float>(steps_completed_)/total,result->success,result->message);} catch(const std::exception &) {}
  if(!shutdown_) {
    ownership_registry_.release(easyfly_core::goalIdentity(handle->get_goal_id()));
    mission_active_=false;
  }
}
bool MissionFSM::execute_step(const MissionStep &s,int index,int total,std::shared_ptr<Handle> handle) {
  if(s.type=="track" && s.duration<=0) {stop("invalid_track_duration"); return false;}
  {std::lock_guard<std::mutex> lock(step_mtx_);current_type_=s.type;}
  const auto start=Clock::now();
  const auto deadline=start+std::chrono::duration<double>(s.timeout);
  if(s.type=="hover") {
    // Wait only: the flight controller continues its existing hold. Legacy
    // ground hover remains a non-flight delay.
    while(std::chrono::duration<double>(Clock::now()-start).count()<s.wait) {
      if(shutdown_ || !reason().empty()) return false;
      if(Clock::now()>=deadline) {stop("hover_timeout"); return false;}
      feedback(phase(s.type),s.type,index,total,static_cast<float>(index)/total,handle);
      std::this_thread::sleep_for(20ms);
    }
    if(Clock::now()>deadline) {stop("hover_timeout");return false;}
    steps_completed_=index+1;
    feedback(phase(s.type),s.type,index+1,total,static_cast<float>(index+1)/total,handle);
    return true;
  }
  if(!sdk_->server_ready(backend(s.type))) {stop("step_backend_unavailable"); return false;}
  std::shared_ptr<easyfly_core::Operation> operation;
  if(s.type=="takeoff") {
    easyfly_msgs::action::Takeoff::Goal g; g.target_altitude=s.altitude; g.climb_speed=s.climb_speed;
    operation=sdk_->takeoff(g);
  } else if(s.type=="waypoint") {
    easyfly_msgs::action::GoToWaypoint::Goal g;
    g.target_x=s.x;g.target_y=s.y;g.target_z=s.z;g.target_yaw=s.yaw;
    g.max_speed=s.max_speed;g.acceptance_radius=s.acceptance_radius;operation=sdk_->go_to(g);
  } else if(s.type=="navigate") {
    easyfly_msgs::action::NavigateTo::Goal g;
    g.target_x=s.x;g.target_y=s.y;g.target_z=s.z;g.max_speed=s.max_speed;operation=sdk_->navigate(g);
  } else if(s.type=="track") {
    easyfly_msgs::action::TrackTarget::Goal g;
    g.target_id=s.target_id;g.follow_distance=s.follow_distance;g.follow_height=s.follow_height;
    operation=sdk_->track(g);
  } else if(s.type=="search") {
    easyfly_msgs::action::SearchArea::Goal g;
    g.center_x=s.center_x;g.center_y=s.center_y;g.area_width=s.area_width;g.area_height=s.area_height;
    g.search_altitude=s.altitude;g.search_speed=s.max_speed;g.search_pattern=s.search_pattern;
    operation=sdk_->search(g);
  } else if(s.type=="explore") {
    easyfly_msgs::action::Explore::Goal g;
    const auto point=[](const std::array<double,3> &p) {
      geometry_msgs::msg::Point value;value.x=p[0];value.y=p[1];value.z=p[2];return value;};
    g.min_bound=point(s.min_bound);g.max_bound=point(s.max_bound);
    for(const auto &box:s.forbidden_boxes) {
      g.forbidden_boxes.push_back(point(box.min));g.forbidden_boxes.push_back(point(box.max));
    }
    g.return_home=s.return_home;g.timeout_sec=s.timeout;operation=sdk_->explore(g);
  } else if(s.type=="precision_land") {
    easyfly_msgs::action::PrecisionLand::Goal g; g.tag_id=s.tag_id; g.timeout=s.timeout;
    operation=sdk_->precision_land(g);
  } else if(s.type=="land") operation=sdk_->land(easyfly_msgs::action::Land::Goal{});
  else {stop("unsupported_step"); return false;}
  {std::lock_guard<std::mutex> lock(step_mtx_);current_=operation;}
  Clock::time_point accepted_at{};
  bool duration_cancel=false;
  double last_feedback_receipt=0;
  const auto relay_feedback=[&](const easyfly_core::OperationSnapshot &snapshot,bool active) {
    if(s.type!="precision_land" && s.type!="explore") return;
    const double receipt=snapshot.feedback_received_seconds;
    const double age=std::chrono::duration<double>(Clock::now().time_since_epoch()).count()-receipt;
    if(snapshot.goal_id.empty() || (active && (receipt<=last_feedback_receipt || age<0 || age>.5))) return;
    easyfly_msgs::msg::MissionStepFeedback event;
    event.header.stamp=now();event.header.frame_id="odom";
    event.mission_goal_id=easyfly_core::goalIdentity(handle->get_goal_id());
    event.child_goal_id=snapshot.goal_id;event.step_type=s.type;event.step_index=index;
    event.active=active;event.feedback_age_sec=active ? age : 0;
    const auto number=[&](const char *key) {
      const auto it=snapshot.feedback.numbers.find(key);
      return it==snapshot.feedback.numbers.end() ? std::numeric_limits<double>::quiet_NaN() : it->second;
    };
    const auto phase=snapshot.feedback.strings.find("phase");
    event.phase=active && phase!=snapshot.feedback.strings.end() ? phase->second : "terminal";
    if(event.phase.size()>128) return;
    if(active && s.type=="precision_land") {
      const auto target=snapshot.feedback.flags.find("target_valid");
      event.target_valid=target!=snapshot.feedback.flags.end() && target->second;
      event.horizontal_error_m=number("horizontal_error_m");
      event.height_above_target_m=number("height_above_target_m");
    } else if(active) {
      event.coverage=number("coverage");
      const double count=number("completed_viewpoints");
      if(!std::isfinite(count) || count<0 || count>std::numeric_limits<uint32_t>::max() || std::floor(count)!=count) return;
      event.completed_viewpoints=static_cast<uint32_t>(count);
      event.next_viewpoint.x=number("next_x");event.next_viewpoint.y=number("next_y");
      event.next_viewpoint.z=number("next_z");
    }
    pub_step_feedback_->publish(event);last_feedback_receipt=receipt;
  };
  while(!shutdown_ && reason().empty()) {
    auto snapshot=operation->snapshot();
    relay_feedback(snapshot,true);
    if(Clock::now()>=deadline) {stop("step_timeout");break;}
    if(snapshot.admission==easyfly_core::Admission::Accepted && accepted_at==Clock::time_point{})
      accepted_at=Clock::now();
    if(!snapshot.remote_execution_pending ||
       (snapshot.action_terminal_received && snapshot.status!=Status::Succeeded)) break;
    if(snapshot.local_status!=easyfly_core::LocalStatus::Active) {stop("child_local_fault");break;}
    const auto u=sdk_->state().value;
    if(u.flight_mode==7 && s.type!="land" && s.type!="precision_land") {
      stop("external_land_takeover");break;
    }
    if(s.type=="track" && s.duration>0 && accepted_at!=Clock::time_point{} &&
       std::chrono::duration<double>(Clock::now()-accepted_at).count()>=s.duration) {
      duration_cancel=true; operation->cancel(); break;
    }
    float fraction=0;
    const auto p=snapshot.feedback.numbers.find("progress");
    if(p!=snapshot.feedback.numbers.end() && std::isfinite(p->second))
      fraction=std::clamp(static_cast<float>(p->second),0.f,.99f);
    feedback(phase(s.type),s.type,index,total,(index+fraction)/total,handle);
    operation->wait(.02);
  }
  if(operation->snapshot().remote_execution_pending) {
    operation->cancel();
    const auto cancel_end=Clock::now()+std::chrono::duration<double>(cancel_budget_s_);
    while(!shutdown_ && operation->snapshot().remote_execution_pending && Clock::now()<cancel_end)
      operation->wait(.02);
  }
  const auto final=operation->snapshot();
  relay_feedback(final,false);
  if(reason().empty() && Clock::now()>deadline) stop("step_timeout");
  const bool ok=reason().empty() && !shutdown_ && !final.remote_execution_pending &&
    (s.type=="track" ? duration_cancel && final.status==Status::Canceled :
      final.status==Status::Succeeded && final.success);
  if(ok) {
    {std::lock_guard<std::mutex> lock(step_mtx_);current_.reset();}
    steps_completed_=index+1;
    feedback(phase(s.type),s.type,index+1,total,static_cast<float>(index+1)/total,handle);
  } else if(reason().empty()) stop(final.remote_execution_pending ? "child_cancel_unresolved" :
    "step_failed:"+final.message);
  return ok;
}
}
