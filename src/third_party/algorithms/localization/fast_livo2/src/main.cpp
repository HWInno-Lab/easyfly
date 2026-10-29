#include "LIVMapper.h"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");
    image_transport::ImageTransport it(nh);
    LIVMapper mapper(nh, "laserMapping");
    mapper.initializeSubscribersAndPublishers(nh, it);
    mapper.run(nh);
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {std::cerr << e.what() << std::endl; result = 1;}
  }
  rclcpp::shutdown();
  return result;
}
