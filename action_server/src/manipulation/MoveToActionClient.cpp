#include "manipulation/MoveToActionClient.hpp"

namespace manipulation
{

MoveToPredefinedActionClient::MoveToPredefinedActionClient()
: Node("moveto_action_client")
{
  this->action_client_ = rclcpp_action::create_client<MoveToPredefined>(
    this, "move_robot_to_predefined");

  auto goal_msg = manipulation_interfaces::action::MoveToPredefined_Goal();

  declare_parameter("group_name", "hand");
  declare_parameter("goal_pose", "open");

  get_parameter("group_name", goal_msg.group_name);
  get_parameter("goal_pose", goal_msg.goal_pose);

  RCLCPP_INFO(
    get_logger(), "Sending goal to move robot arm (%s) to pose %s",
    goal_msg.group_name.c_str(), goal_msg.goal_pose.c_str());

  send_goal(goal_msg);
}

void
MoveToPredefinedActionClient::send_goal(const MoveToPredefined::Goal & goal_msg)
{

  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
  }

  RCLCPP_INFO(get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<MoveToPredefined>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(
    &MoveToPredefinedActionClient::goal_response_callback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &MoveToPredefinedActionClient::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &MoveToPredefinedActionClient::result_callback, this,
    std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

}
void
MoveToPredefinedActionClient::goal_response_callback(
  const GoalHandleMoveToPredefined::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void
MoveToPredefinedActionClient::feedback_callback(
  GoalHandleMoveToPredefined::SharedPtr,
  const std::shared_ptr<const MoveToPredefined::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->msg.c_str());
}

void
MoveToPredefinedActionClient::result_callback(
  const GoalHandleMoveToPredefined::WrappedResult & result)
{
  RCLCPP_INFO(this->get_logger(), "Action finished");
}

} // namespace manipulation
