#include "rclcpp/rclcpp.hpp"
#include <plan_manage/ego_replan_fsm.h>
#include <pthread.h>
#include <signal.h>
#include <cerrno>
#include <chrono>
#include <stdexcept>

using namespace ego_planner;

int main(int argc, char **argv)
{
  // This executable owns shutdown. Block before ROS creates any threads, so
  // SIGINT/SIGTERM cannot invalidate the context during endpoint construction
  // or concurrently with executor/node destruction. Pending requests are
  // consumed by the main thread, including requests received during init.
  sigset_t stop_signals;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGINT);
  sigaddset(&stop_signals, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &stop_signals, nullptr) != 0) {
    throw std::runtime_error("could not block planner shutdown signals");
  }
  bool stopped = false;
  auto stop_requested = [&stop_signals, &stopped]() {
    if (stopped) {return true;}
    const timespec immediate{0, 0};
    const int received = sigtimedwait(&stop_signals, nullptr, &immediate);
    if (received == SIGINT || received == SIGTERM) {stopped = true; return true;}
    if (received < 0 && (errno == EAGAIN || errno == EINTR)) {return false;}
    throw std::runtime_error("could not poll planner shutdown signals");
  };

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  {
    auto nh = std::make_shared<rclcpp::Node>("ego_planner_node");
    EGOReplanFSM rebo_replan;
    rebo_replan.init(nh, stop_requested);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    RCLCPP_INFO(nh->get_logger(), "EGO runtime ready; main-thread shutdown enabled");
    while (rclcpp::ok() && !stop_requested()) {
      // Same single-threaded callbacks; bound only idle waiting for a pending
      // process stop. Running callbacks finish before owned state is released.
      executor.spin_once(std::chrono::milliseconds(50));
    }
    executor.remove_node(nh);
  }  // Executor, FSM and node retire while the context is still valid.
  rclcpp::shutdown();
  return 0;
}
