#include "manipulation/MoveToActionServer.hpp"

namespace manipulation
{

MoveToActionServer::MoveToActionServer()
: Node("manipulation_action_server")
{
  action_server_ = rclcpp_action::create_server<MoveTo>(
    this,
    "move_robot_arm",
    std::bind(&MoveToActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MoveToActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&MoveToActionServer::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Ready to receive goals");

}

rclcpp_action::GoalResponse
MoveToActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const manipulation_action::action::MoveTo::Goal> goal)
{
  goal_ = goal->goal_pose;
  group_ = goal->group_name;
  
  RCLCPP_INFO(this->get_logger(), "Received goal to move robot art to pose %s", goal_.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MoveToActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulation_action::action::MoveTo>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal cancelled");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
MoveToActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMove> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted");
  
  std::thread{std::bind(&MoveToActionServer::move_robot_arm, this, std::placeholders::_1),
    goal_handle}.detach();

}

void
MoveToActionServer::move_robot_arm(const std::shared_ptr<GoalHandleMove> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  // auto feedback = std::make_shared<MoveTo::Feedback>();
  auto result = std::make_shared<MoveTo::Result>();

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<MTCNode>(options, group_, goal_);
  RCLCPP_INFO(this->get_logger(), "MTCNode node created with (%s-%s)",
    group_.c_str(), goal_.c_str());
  rclcpp::executors::MultiThreadedExecutor executor;
  
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->get_node_base_interface());
    executor.spin();
    executor.remove_node(mtc_node->get_node_base_interface());
  });


  RCLCPP_INFO(this->get_logger(), "MTCNode added to executor and spinning...");
  mtc_node->do_task();

  spin_thread->join();
  
  result->success = true;
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  goal_handle->succeed(result);
}

} // namespace manipulation
