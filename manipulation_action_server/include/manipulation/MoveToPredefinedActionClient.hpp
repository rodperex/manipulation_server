#ifndef MANIPULATION__MOVETOPREDEFINEDACTIONCLIENT_HPP_
#define MANIPULATION__MOVETOPREDEFINEDACTIONCLIENT_HPP_

#include <memory>

#include "manipulation_interfaces/action/move_to_predefined.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executor.hpp"

namespace manipulation
{

class MoveToPredefinedActionClient : public rclcpp::Node
{
public:
  using MoveToPredefined = manipulation_interfaces::action::MoveToPredefined;
  using GoalHandleMoveToPredefined = rclcpp_action::ClientGoalHandle<MoveToPredefined>;

  MoveToPredefinedActionClient();

private:
  void send_goal(const MoveToPredefined::Goal & goal_msg);
  void goal_response_callback(const GoalHandleMoveToPredefined::SharedPtr & goal_handle);
  void feedback_callback(
    GoalHandleMoveToPredefined::SharedPtr,
    const std::shared_ptr<const MoveToPredefined::Feedback> feedback);
  void result_callback(const GoalHandleMoveToPredefined::WrappedResult & result);

  rclcpp_action::Client<MoveToPredefined>::SharedPtr action_client_;
};

} // end namespace manipulation

#endif // MANIPULATION__MOVETOPREDEFINEDACTIONCLIENT_HPP_
