#include "rclcpp/rclcpp.hpp"
#include "manipulation/MoveEndEffectorActionClient.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation::MoveEndEffectorActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
