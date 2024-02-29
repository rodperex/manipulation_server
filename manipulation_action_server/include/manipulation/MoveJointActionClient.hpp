#ifndef MANIPULATION__MOVEJOINTACTIONCLIENT_HPP_
#define MANIPULATION__MOVEJOINTACTIONCLIENT_HPP_

#include <memory>

#include "manipulation_interfaces/action/move_joint.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executor.hpp"

namespace manipulation
{

class MoveJointActionClient : public rclcpp::Node
{
public:
  using MoveJoint = manipulation_interfaces::action::MoveJoint;
  using GoalHandleMoveJoint = rclcpp_action::ClientGoalHandle<MoveJoint>;

  MoveJointActionClient();

private:
  void send_goal(const MoveJoint::Goal & goal_msg);
  void goal_response_callback(const GoalHandleMoveJoint::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleMoveJoint::SharedPtr,
    const std::shared_ptr<const MoveJoint::Feedback> feedback);
  void result_callback(const GoalHandleMoveJoint::WrappedResult & result);

  rclcpp_action::Client<MoveJoint>::SharedPtr action_client_;
};

} // end namespace manipulation

#endif // MANIPULATION__MOVEJOINTACTIONCLIENT_HPP_
