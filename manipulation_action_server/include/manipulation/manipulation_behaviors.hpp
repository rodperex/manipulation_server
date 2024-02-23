#ifndef MANIPULATION__MANIPULATION_BEHAVIORS_HPP
#define MANIPULATION__MANIPULATION_BEHAVIORS_HPP


#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <vector>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include "manipulation_interfaces/action/move_to_predefined.hpp"
#include "manipulation_interfaces/action/pick.hpp"
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace manipulation
{

using MoveToPredefined = manipulation_interfaces::action::MoveToPredefined;
using GoalHandleMoveToPredefined = rclcpp_action::ServerGoalHandle<MoveToPredefined>;

using Pick = manipulation_interfaces::action::Pick;
using GoalHandlePick = rclcpp_action::ServerGoalHandle<Pick>;

void ExecuteMoveToPredefined(
    const std::shared_ptr<GoalHandleMoveToPredefined> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner);
void ExecutePick(
    const std::shared_ptr<GoalHandlePick> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner,
    std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner,
    moveit::planning_interface::PlanningSceneInterface psi);
bool EvaluateJoint(
    const std::map<std::string, double>& desired_joint_values,
    const std::vector<double>& tolerances,
    const rclcpp::Node::SharedPtr& node);

moveit::task_constructor::Task ConfigureTask(const std::string& task_name, rclcpp::Node::SharedPtr node);

} // end namespace manipulation

#endif // MANIPULATION__MANIPULATION_BEHAVIORS_HPP
