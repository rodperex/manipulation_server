#ifndef MANIPULATION__MOVETOACTIONCLIENT_HPP_
#define MANIPULATION__MOVETOACTIONCLIENT_HPP_

#include <memory>

#include "manipulation_action/action/move_to.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executor.hpp"

namespace manipulation
{

class MoveToActionClient : public rclcpp::Node
{
public:
    using MoveTo = manipulation_action::action::MoveTo;
    using GoalHandleMoveTo = rclcpp_action::ClientGoalHandle<MoveTo>;

    MoveToActionClient();

private:
    void send_goal(const MoveTo::Goal& goal_msg);
    void goal_response_callback(const GoalHandleMoveTo::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleMoveTo::SharedPtr,
        const std::shared_ptr<const MoveTo::Feedback> feedback);
    void result_callback(const GoalHandleMoveTo::WrappedResult & result);
    
    rclcpp_action::Client<MoveTo>::SharedPtr action_client_;
};

} // end namespace manipulation

#endif // MANIPULATION__MOVETOACTIONCLIENT_HPP_
