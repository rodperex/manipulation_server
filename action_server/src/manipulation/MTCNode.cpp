#include "manipulation/MTCNode.hpp"


namespace manipulation
{

MTCNode::MTCNode(const rclcpp::NodeOptions & options, std::string group, std::string goal)
: group_(group),
  goal_(goal)
{
  node_ = std::make_shared<rclcpp::Node>("moveit_task_constructor_node", options);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCNode::get_node_base_interface()
{
  return node_->get_node_base_interface();
}

void
MTCNode::do_task()
{
  task_ = create_task();

  try {
    task_.init();
  } catch (moveit::task_constructor::InitStageException & e) {
    RCLCPP_ERROR(node_->get_logger(), "Initialization failed: %s", e.what());
    return;
  }

  if (!task_.plan(5)) { // creates 5 solutions
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
    return;
  }
  // this for visualization in RViz
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Task execution failed");
    return;
  }

  return;
}

moveit::task_constructor::Task
MTCNode::create_task()
{
  moveit::task_constructor::Task task;

  task.stages()->setName("move to pose");
  task.loadRobotModel(node_);


  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(
    "current");
  task.add(std::move(stage_state_current));

  auto interpolation_planner =
    std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

  auto stage =
    std::make_unique<moveit::task_constructor::stages::MoveTo>("given pose", interpolation_planner);

  stage->setGroup(group_);
  stage->setGoal(goal_);

  task.add(std::move(stage));

  return task;

}
} // namespace manipulation
