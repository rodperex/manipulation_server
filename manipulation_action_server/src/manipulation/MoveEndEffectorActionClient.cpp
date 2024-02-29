#include "manipulation/MoveEndEffectorActionClient.hpp"

namespace manipulation
{

MoveEndEffectorActionClient::MoveEndEffectorActionClient()
: Node("manipulation_client")
{
  this->action_client_ = rclcpp_action::create_client<MoveEndEffector>(
    this, "move_end_effector");

  auto goal_msg = manipulation_interfaces::action::MoveEndEffector_Goal();

  geometry_msgs::msg::TransformStamped tr;

  declare_parameter("x_eef", 0.0);
  declare_parameter("y_eef", 0.0);
  declare_parameter("z_eef", 0.0);
  get_parameter("x_eef", tr.transform.translation.x);
  get_parameter("y_eef", tr.transform.translation.y);
  get_parameter("z_eef", tr.transform.translation.z);
  
  goal_msg.eef2goal = tr;

  RCLCPP_INFO(
    get_logger(), "Sending goal to move end effector to pose <%.2f,%.2f,%.2f>",
      tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z);

  send_goal(goal_msg);
}

void
MoveEndEffectorActionClient::send_goal(const MoveEndEffector::Goal & goal_msg)
{

  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
  }

  RCLCPP_INFO(get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<MoveEndEffector>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(
    &MoveEndEffectorActionClient::goal_response_callback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &MoveEndEffectorActionClient::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &MoveEndEffectorActionClient::result_callback, this,
    std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

}
void
MoveEndEffectorActionClient::goal_response_callback(
  const GoalHandleMoveEndEffector::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void
MoveEndEffectorActionClient::feedback_callback(
  GoalHandleMoveEndEffector::SharedPtr,
  const std::shared_ptr<const MoveEndEffector::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->msg.c_str());
}

void
MoveEndEffectorActionClient::result_callback(
  const GoalHandleMoveEndEffector::WrappedResult & result)
{
  RCLCPP_INFO(this->get_logger(), "Action finished");
}

} // namespace manipulation
