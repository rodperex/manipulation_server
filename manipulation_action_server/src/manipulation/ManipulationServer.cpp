#include "manipulation/ManipulationServer.hpp"
#include <algorithm>
#include <vector>

namespace manipulation
{
// todos:
// add parameters
// able to prempt the task
// add the rest actions
ManipulationServer::ManipulationServer(const rclcpp::NodeOptions & options)
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("manipulation_action_server", "", options),
  node_{std::make_shared<rclcpp::Node>("manipulation_action_server_executor", options)}
{

  RCLCPP_INFO(this->get_logger(), "Manipulation server created");

}
ManipulationServer::~ManipulationServer()
{
}

CallbackReturn
ManipulationServer::on_configure(const rclcpp_lifecycle::State & state)
{
  // this is for GETTING parameters (todo)
  RCLCPP_INFO(get_logger(), "Configuring manipulation server");
  action_server_predefined_ = rclcpp_action::create_server<MoveToPredefined>(
    this,
    "move_to_predefined",
    std::bind(
      &ManipulationServer::handle_move_to_predefined_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&ManipulationServer::handle_move_to_predefined_cancel, this, std::placeholders::_1),
    std::bind(&ManipulationServer::handle_move_to_predefined_accepted, this, std::placeholders::_1)
  );

  action_server_pick_ = rclcpp_action::create_server<Pick>(
    this,
    "pick",
    std::bind(&ManipulationServer::handle_pick_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ManipulationServer::handle_pick_cancel, this, std::placeholders::_1),
    std::bind(&ManipulationServer::handle_pick_accepted, this, std::placeholders::_1)
  );

  interpolation_planner_ =
    std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  // change the name to be a parameter
  // task_.stages()->setName("demo task");
  // task_.loadRobotModel(node_);
  // task_.setProperty("ik_frame", "<gr>ipper_grasping_frame");

  // task_ = ConfigureTask("manipulation_task", node_);
  *task_ = ConfigureTask("manipulation_task", node_);

  cartesian_planner_ =
    std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(1.0);
  cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner_->setStepSize(.01);

  sampling_planner_ =
    std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);

  node_thread_ = std::make_unique<std::thread>(
    [this]() {
      executor_.add_node(node_);
      // executor_.spin();
      while(!should_exit_){
        executor_.spin_some();
      }
      executor_.remove_node(node_);
    });

  // Why?
  // stage_state_current_ =
  //   std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
  // current_state_ptr_ = stage_state_current_.get();


  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleanning up manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  if (node_thread_->joinable()) {
    node_thread_->join();
    RCLCPP_INFO(get_logger(), "Thread joining...");
  }
  RCLCPP_INFO(get_logger(), "Shutting down manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse
ManipulationServer::handle_move_to_predefined_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const manipulation_interfaces::action::MoveToPredefined::Goal> goal)
{

  RCLCPP_INFO(this->get_logger(), "Received goal: move arm to predefined pose %s", goal->goal_pose.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse
ManipulationServer::handle_pick_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const manipulation_interfaces::action::Pick::Goal> goal)
{

  RCLCPP_INFO(this->get_logger(), "Received goal: pick %s", goal->object_goal.id.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse
ManipulationServer::handle_place_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const manipulation_interfaces::action::Place::Goal> goal)
{

  RCLCPP_INFO(this->get_logger(), "Received goal: place %s", goal->attached_object.id.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ManipulationServer::handle_move_to_predefined_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulation_interfaces::action::MoveToPredefined>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal: move arm to predefined pose");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse
ManipulationServer::handle_pick_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulation_interfaces::action::Pick>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal: pick");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse
ManipulationServer::handle_place_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulation_interfaces::action::Place>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal: place");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ManipulationServer::handle_move_to_predefined_accepted(
  const std::shared_ptr<GoalHandleMoveToPredefined> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted (move_to_predefined): %s", goal_handle->get_goal()->goal_pose.c_str());

  // task_thread_ = std::make_unique<std::thread>(
  //       std::bind(&ManipulationServer::execute_move_to_predefined, this, std::placeholders::_1),
  //       goal_handle
  // );
  // task_thread_->detach();
  ExecuteMoveToPredefined(goal_handle, node_, interpolation_planner_);

}

void
ManipulationServer::handle_pick_accepted(
  const std::shared_ptr<GoalHandlePick> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted (pick): %s", goal_handle->get_goal()->object_goal.id.c_str());

  attach_object_stage_ptr_ = 
    ExecutePick(goal_handle,
      node_,
      interpolation_planner_,
      cartesian_planner_,
      planning_interface_,
      task_);
}

void
ManipulationServer::handle_place_accepted(
  const std::shared_ptr<GoalHandlePlace> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted (place): %s", goal_handle->get_goal()->attached_object.id.c_str());

  ExecutePlace(goal_handle,
    node_,
    interpolation_planner_,
    cartesian_planner_,
    sampling_planner_,
    planning_interface_,
    attach_object_stage_ptr_,
    task_);
}

} // namespace manipulation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(manipulation::ManipulationServer)
