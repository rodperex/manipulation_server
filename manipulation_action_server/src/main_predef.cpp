#include "rclcpp/rclcpp.hpp"
#include "manipulation/MoveToPredefinedActionClient.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation::MoveToPredefinedActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
