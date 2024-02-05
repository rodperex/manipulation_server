#ifndef MANIPULATION__MOVETOACTIONSERVER_HPP
#define MANIPULATION__MOVETOACTIONSERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "manipulation_action/action/move_to.hpp" 

#include "manipulation/MTCNode.hpp"


namespace manipulation
{

class MoveToActionServer : public rclcpp::Node
{
public:
    using MoveTo = manipulation_action::action::MoveTo;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveTo>;

    MoveToActionServer();

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveTo::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveTo>> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleMove> goal_handle);
    
    void move_robot_arm(const std::shared_ptr<GoalHandleMove> goal_handle);
    
    rclcpp_action::Server<MoveTo>::SharedPtr action_server_;

    std::string group_, goal_;
};

} // end namespace manipulation

#endif // MANIPULATION__MOVETOACTIONSERVER_HPP
