#include "rclcpp/rclcpp.hpp"
#include "manipulation/MoveJointActionClient.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation::MoveJointActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
