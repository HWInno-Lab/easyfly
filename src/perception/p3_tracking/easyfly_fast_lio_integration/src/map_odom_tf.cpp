// EasyFly M3 定制: map→odom TF 发布器 (REP-105)
//
// 背景: FAST-LIO 发布其世界系 (camera_init) Odometry, 但不发布 map→odom TF。
// EasyFly TF 树要求: map → odom → base_link (M1 的 odom→base_link 由
// easyfly_coord_transform 维护)。
//
// 原理: 每收到一帧 SLAM Odometry (T_map_body), 先转换到 base_link，结合 /odom，
// 计算 T_map_odom = T_map_base × T_odom_base⁻¹ 并发布静态化的 TF。
//   - 两个 odom 帧同源同时刻时, T_map_odom 理论恒定
//   - 实际存在漂移差异 → 用低通滤波平滑, 避免跳变传入控制链
//
// 独立节点设计 (不侵入 fast_lio 源码, 便于上游同步)。

#include <memory>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/odometry.hpp>

namespace easyfly_fast_lio_integration
{

class MapOdomTf : public rclcpp::Node
{
public:
    explicit MapOdomTf(const rclcpp::NodeOptions & options)
        : Node("map_odom_tf", options)
    {
        // SLAM 世界系名 (FAST-LIO 输出 frame, 默认 camera_init)
        declare_parameter<std::string>("slam_frame", "camera_init");
        declare_parameter<std::string>("odom_topic", "/odom");
        declare_parameter<std::string>("slam_topic", "/Odometry");
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("odom_frame", "odom");
        // 平滑系数 (0~1, 越小越平滑)
        declare_parameter<double>("smooth_alpha", 0.1);
        // Translation of the base origin in SLAM body coordinates; axes must align.
        // Default preserves native setups. Gazebo explicitly selects its IMU offset.
        rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
        const auto body_to_base = declare_parameter<std::vector<double>>(
            "body_to_base_translation", {0.0, 0.0, 0.0}, fixed);
        if (body_to_base.size() != 3) {
            throw std::invalid_argument("body_to_base_translation requires 3 values");
        }
        for (const auto value : body_to_base) {
            if (!std::isfinite(value) || std::abs(value) > 100) {
                throw std::invalid_argument("invalid body_to_base_translation");
            }
        }
        body_to_base_.setIdentity();
        body_to_base_.setOrigin(tf2::Vector3(body_to_base[0], body_to_base[1], body_to_base[2]));

        slam_frame_ = get_parameter("slam_frame").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        alpha_ = get_parameter("smooth_alpha").as_double();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // 修复 (2026-09-13): 同时发布 map → slam_frame (camera_init) 恒等静态 TF。
        // FAST-LIO 的 /cloud_registered 等话题 frame 是 camera_init, 而 RViz
        // Fixed Frame 是 map — 缺少此桥接时 TF 树分成两棵不连通子树
        // (map→odom→base_link 与 camera_init→body), 点云无法显示。
        // SLAM 世界系 (camera_init) 即本节点所维护的 map 原点, 恒等正确。
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped id_tf;
        id_tf.header.frame_id = map_frame_;
        id_tf.child_frame_id = slam_frame_;
        id_tf.header.stamp = this->now();
        id_tf.transform.rotation.w = 1.0;   // 恒等变换
        static_tf_broadcaster_->sendTransform(id_tf);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("odom_topic").as_string(), 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                odom_ = msg;
            });

        slam_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("slam_topic").as_string(), 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onSlam(msg); });

        RCLCPP_INFO(get_logger(),
                    "map_odom_tf 启动: %s → %s (slam=%s, 平滑系数=%.2f)",
                    map_frame_.c_str(), odom_frame_.c_str(), slam_frame_.c_str(), alpha_);
    }

private:
    static tf2::Transform toTf(const nav_msgs::msg::Odometry & o)
    {
        tf2::Transform t;
        const auto & p = o.pose.pose.position;
        const auto & q = o.pose.pose.orientation;
        t.setOrigin(tf2::Vector3(p.x, p.y, p.z));
        t.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
        return t;
    }

    void onSlam(const nav_msgs::msg::Odometry::ConstSharedPtr & slam)
    {
        if (!odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "尚未收到 /odom (easyfly_coord_transform 未运行?)");
            return;
        }

        // FAST-LIO body is the IMU sampling origin, not necessarily base_link.
        const tf2::Transform T_map_base = toTf(*slam) * body_to_base_;
        const tf2::Transform T_odom_base = toTf(*odom_);

        // T_map_odom = T_map_base × T_odom_base⁻¹
        const tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        if (!has_init_) {
            tf_est_ = T_map_odom;
            has_init_ = true;
        } else {
            // 指数平滑 (SLAM 与 mavros 漂移差异的缓变跟踪)
            const tf2::Vector3 & o_new = T_map_odom.getOrigin();
            const tf2::Vector3 & o_old = tf_est_.getOrigin();
            tf2::Vector3 o_smooth = o_old + alpha_ * (o_new - o_old);

            // 旋转用 slerp 近似: 四元数插值
            tf2::Quaternion q_new = T_map_odom.getRotation().normalize();
            tf2::Quaternion q_old = tf_est_.getRotation().normalize();
            tf2::Quaternion q_s = q_old.slerp(q_new, alpha_).normalize();

            tf_est_.setOrigin(o_smooth);
            tf_est_.setRotation(q_s);
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = slam->header.stamp;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = odom_frame_;
        tf_msg.transform.translation.x = tf_est_.getOrigin().x();
        tf_msg.transform.translation.y = tf_est_.getOrigin().y();
        tf_msg.transform.translation.z = tf_est_.getOrigin().z();
        tf_msg.transform.rotation.x = tf_est_.getRotation().x();
        tf_msg.transform.rotation.y = tf_est_.getRotation().y();
        tf_msg.transform.rotation.z = tf_est_.getRotation().z();
        tf_msg.transform.rotation.w = tf_est_.getRotation().w();
        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::string slam_frame_, map_frame_, odom_frame_;
    double alpha_{0.1};
    bool has_init_{false};
    tf2::Transform tf_est_;
    tf2::Transform body_to_base_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, slam_sub_;
};

}  // namespace easyfly_fast_lio_integration

RCLCPP_COMPONENTS_REGISTER_NODE(easyfly_fast_lio_integration::MapOdomTf)
