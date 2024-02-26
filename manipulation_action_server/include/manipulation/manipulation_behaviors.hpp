#ifndef MANIPULATION__MANIPULATION_BEHAVIORS_HPP
#define MANIPULATION__MANIPULATION_BEHAVIORS_HPP

#include <map>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <vector>
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include "manipulation_interfaces/action/move_to_predefined.hpp"
#include "manipulation_interfaces/action/move_joint.hpp"
#include "manipulation_interfaces/action/pick.hpp"
#include "manipulation_interfaces/action/place.hpp"
#include "manipulation_interfaces/action/pick_and_place.hpp"
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

using MoveJoint = manipulation_interfaces::action::MoveJoint;
using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;

using Pick = manipulation_interfaces::action::Pick;
using GoalHandlePick = rclcpp_action::ServerGoalHandle<Pick>;

using Place = manipulation_interfaces::action::Place;
using GoalHandlePlace = rclcpp_action::ServerGoalHandle<Place>;

using PickAndPlace = manipulation_interfaces::action::PickAndPlace;
using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

moveit::task_constructor::Task MoveToPredefinedTask(
    std::string group_name,
    std::string goal_pose,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner>
        interpolation_planner);

moveit::task_constructor::Task MoveJointTask(
    std::string joint_name,
    geometry_msgs::msg::Pose goal_pose,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::task_constructor::solvers::CartesianPath>
        interpolation_planner);

moveit::task_constructor::Task PickTask(
    moveit_msgs::msg::CollisionObject object,
    moveit::task_constructor::Stage*& attach_object_stage,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner>
        interpolation_planner,
    std::shared_ptr<moveit::task_constructor::solvers::CartesianPath>
        cartesian_planner,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
        psi);

moveit::task_constructor::Task PlaceTask(
    moveit_msgs::msg::CollisionObject object,
    geometry_msgs::msg::Pose place_pose,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner>
        interpolation_planner,
    std::shared_ptr<moveit::task_constructor::solvers::CartesianPath>
        cartesian_planner,
    std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
        sampling_planner,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
        psi);

moveit::task_constructor::Task PickAndPlaceTask(
    moveit_msgs::msg::CollisionObject object,
    geometry_msgs::msg::Pose place_pose,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner>
        interpolation_planner,
    std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> 
        cartesian_planner,
    std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
        sampling_planner,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
        psi);

bool EvaluateJoint(
    const std::map<std::string, double>& desired_joint_values,
    const std::vector<double>& tolerances);

bool ExecuteTask(
    moveit::task_constructor::Task& task,
    rclcpp::Node::SharedPtr node);

bool IsGripperClosed(rclcpp::Node::SharedPtr node);

} // end namespace manipulation

#endif // MANIPULATION__MANIPULATION_BEHAVIORS_HPP
