#ifndef MANIPULATION__MOVETOACTIONSERVER_HPP
#define MANIPULATION__MOVETOACTIONSERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <algorithm>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "cascade_lifecycle_msgs/msg/activation.hpp"
#include "cascade_lifecycle_msgs/msg/state.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "manipulation/manipulation_behaviors.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

namespace manipulation
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ManipulationServer : public  rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:

  ManipulationServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
  );
  virtual ~ManipulationServer();

  CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void execute_move_to_predefined(
    const std::shared_ptr<GoalHandleMoveToPredefined> goal_handle);
  void execute_move_joint(
    const std::shared_ptr<GoalHandleMoveJoint> goal_handle);
  void execute_pick(
    const std::shared_ptr<GoalHandlePick> goal_handle);
  void execute_pick_and_place(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);
  void execute_place(
    const std::shared_ptr<GoalHandlePlace> goal_handle);

  rclcpp_action::Server<MoveToPredefined>::SharedPtr
    action_server_move_to_predefined_;
  rclcpp_action::Server<MoveJoint>::SharedPtr
    action_server_move_joint_;
  rclcpp_action::Server<Pick>::SharedPtr
    action_server_pick_;
  rclcpp_action::Server<PickAndPlace>::SharedPtr
    action_server_pick_and_place_;
  rclcpp_action::Server<Place>::SharedPtr
    action_server_place_;
  
  rclcpp_action::GoalResponse handle_move_to_predefined_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPredefined::Goal> goal);
  rclcpp_action::GoalResponse handle_move_joint_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJoint::Goal> goal);
  rclcpp_action::GoalResponse handle_pick_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Pick::Goal> goal);
  rclcpp_action::GoalResponse handle_pick_and_place_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickAndPlace::Goal> goal);
  rclcpp_action::GoalResponse handle_place_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Place::Goal> goal);  

  rclcpp_action::CancelResponse handle_move_to_predefined_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPredefined>> goal_handle);
  rclcpp_action::CancelResponse handle_move_joint_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJoint>> goal_handle);
  rclcpp_action::CancelResponse handle_pick_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>> goal_handle);
  rclcpp_action::CancelResponse handle_pick_and_place_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAndPlace>> goal_handle);
  rclcpp_action::CancelResponse handle_place_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Place>> goal_handle);

  void handle_move_to_predefined_accepted(
    const std::shared_ptr<GoalHandleMoveToPredefined> goal_handle);
  void handle_move_joint_accepted(
    const std::shared_ptr<GoalHandleMoveJoint> goal_handle);
  void handle_pick_accepted(
    const std::shared_ptr<GoalHandlePick> goal_handle);
  void handle_pick_and_place_accepted(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle);
  void handle_place_accepted(
    const std::shared_ptr<GoalHandlePlace> goal_handle);

  moveit::task_constructor::Task task_;
  
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner>
    interpolation_planner_;
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath>
    cartesian_planner_;
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>
    sampling_planner_;

  // Not sure whether this should be a pointer or not (are we just connecting to moveit2?)
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_interface_;
  moveit::task_constructor::Stage * attach_object_stage_ptr_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<std::thread> node_thread_;
  // std::unique_ptr<std::thread> task_thread_;

  std::atomic<bool> should_exit_{false};
  bool has_picked_{false};

};

} // end namespace manipulation

#endif // MANIPULATION__MOVETOACTIONSERVER_HPP
