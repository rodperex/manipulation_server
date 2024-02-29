#include "manipulation/MoveJointActionClient.hpp"

namespace manipulation
{

MoveJointActionClient::MoveJointActionClient()
: Node("manipulation_client")
{
  this->action_client_ = rclcpp_action::create_client<MoveJoint>(
    this, "move_joint");

  auto goal_msg = manipulation_interfaces::action::MoveJoint_Goal();

  declare_parameter("group_name", "");
  declare_parameter("joint_name", "");
  declare_parameter("joint_value", 0.0);
  
  
  get_parameter("group_name", goal_msg.group_name);
  get_parameter("joint_name", goal_msg.joint_name);
  get_parameter("joint_value", goal_msg.joint_value);

  RCLCPP_INFO(
    get_logger(), "Sending goal to move %s to value %.2f",
      goal_msg.joint_name.c_str(), goal_msg.joint_value);

  send_goal(goal_msg);
}

void
MoveJointActionClient::send_goal(const MoveJoint::Goal & goal_msg)
{

  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
  }

  RCLCPP_INFO(get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<MoveJoint>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(
    &MoveJointActionClient::goal_response_callback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &MoveJointActionClient::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &MoveJointActionClient::result_callback, this,
    std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

}
void
MoveJointActionClient::goal_response_callback(
  const GoalHandleMoveJoint::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void
MoveJointActionClient::feedback_callback(
  GoalHandleMoveJoint::SharedPtr,
  const std::shared_ptr<const MoveJoint::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->msg.c_str());
}

void
MoveJointActionClient::result_callback(
  const GoalHandleMoveJoint::WrappedResult & result)
{
  RCLCPP_INFO(this->get_logger(), "Action finished");
}

} // namespace manipulation
