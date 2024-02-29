#ifndef MANIPULATION__MOVEENDEFFECTORACTIONCLIENT_HPP_
#define MANIPULATION__MOVEENDEFFECTORACTIONCLIENT_HPP_

#include <memory>

#include "manipulation_interfaces/action/move_end_effector.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executor.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace manipulation
{

class MoveEndEffectorActionClient : public rclcpp::Node
{
public:
  using MoveEndEffector = manipulation_interfaces::action::MoveEndEffector;
  using GoalHandleMoveEndEffector = rclcpp_action::ClientGoalHandle<MoveEndEffector>;

  MoveEndEffectorActionClient();

private:
  void send_goal(const MoveEndEffector::Goal & goal_msg);
  void goal_response_callback(const GoalHandleMoveEndEffector::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleMoveEndEffector::SharedPtr,
    const std::shared_ptr<const MoveEndEffector::Feedback> feedback);
  void result_callback(const GoalHandleMoveEndEffector::WrappedResult & result);

  rclcpp_action::Client<MoveEndEffector>::SharedPtr action_client_;
};

} // end namespace manipulation

#endif // MANIPULATION__MOVEENDEFFECTORACTIONCLIENT_HPP_
