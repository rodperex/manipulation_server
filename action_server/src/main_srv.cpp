#include "rclcpp/rclcpp.hpp"
#include "manipulation/MoveToActionServer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<manipulation::MoveToActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}