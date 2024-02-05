#include "manipulation/MoveToActionClient.hpp"

namespace manipulation
{

MoveToActionClient::MoveToActionClient()
: Node("moveto_action_client")
{
  this->action_client_ = rclcpp_action::create_client<MoveTo>(
      this, "move_robot_arm_client");
  
  auto goal_msg = manipulation_action::action::MoveTo_Goal();

  declare_parameter("group_name", "hand");
  declare_parameter("goal_pose", "open");

  get_parameter("group_name", goal_msg.group_name);
  get_parameter("goal_pose", goal_msg.goal_pose);

  send_goal(goal_msg);
}

void
MoveToActionClient::send_goal(const MoveTo::Goal& goal_msg)
{
  
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
  }

  RCLCPP_INFO(get_logger(), "Sending goal");
  
  auto send_goal_options = rclcpp_action::Client<MoveTo>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&MoveToActionClient::goal_response_callback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&MoveToActionClient::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&MoveToActionClient::result_callback, this,
    std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

}
void
MoveToActionClient::goal_response_callback(const GoalHandleMoveTo::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void
MoveToActionClient::feedback_callback(GoalHandleMoveTo::SharedPtr,
  const std::shared_ptr<const MoveTo::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->msg.c_str());
}

void
MoveToActionClient::result_callback(const GoalHandleMoveTo::WrappedResult & result)
{
  RCLCPP_INFO(this->get_logger(), "Action finished");
}

} // namespace manipulation
