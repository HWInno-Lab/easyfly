// SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
// SPDX-FileCopyrightText: 2026 贾国源
// License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_rc_receiver/licenses/.
// This revision offers no Apache-2.0 option for the listed original code.
/**
 * rc_receiver_node.cpp — 从 mavros2 /mavros/rc/in 读取遥控器通道值
 *
 * 输入: /mavros/rc/in (mavros_msgs/RCIn)
 * 输出: /rc_channels (std_msgs/Float32MultiArray) — 原始 PWM 值 [1000~2000]
 *
 * 阶段一: 完全依赖 mavros2，不直连 SBUS 硬件。
 */
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class RcReceiver : public rclcpp::Node
{
public:
  RcReceiver()
    : Node("rc_receiver")
  {
    sub_rc_ = create_subscription<mavros_msgs::msg::RCIn>(
      "/mavros/rc/in", rclcpp::SensorDataQoS(),
      [this](const mavros_msgs::msg::RCIn::SharedPtr msg) {
        auto out = std_msgs::msg::Float32MultiArray();
        out.data.reserve(msg->channels.size());
        for (auto ch : msg->channels) {
          out.data.push_back(static_cast<float>(ch));
        }
        pub_->publish(out);
      });

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/rc_channels", rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(), "RcReceiver started — /mavros/rc/in → /rc_channels");
  }

private:
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr sub_rc_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RcReceiver>());
  rclcpp::shutdown();
  return 0;
}
