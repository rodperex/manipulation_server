#include "manipulation/manipulation_behaviors.hpp"
#include <map>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <algorithm>
#include <chrono>


namespace manipulation
{

using namespace std::chrono_literals;

moveit::task_constructor::Task ConfigureTask(const std::string& task_name, rclcpp::Node::SharedPtr node)
{
  moveit::task_constructor::Task task;

  task.stages()->setName(task_name);
  task.loadRobotModel(node);

  // let properties be a node parameter
  task.setProperty("ik_frame", "gripper_grasping_frame");
  task.setProperty("group", "arm_torso");
  task.setProperty("eef", "gripper");

  return task;
}

bool SendTask(moveit::task_constructor::Task& task, rclcpp::Node::SharedPtr node)
{
  try {
    task.init();
  } catch (moveit::task_constructor::InitStageException & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e);
    return false;
  }
  if (!task.plan(5)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Task planning failed");
    return false;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Task planning succeeded, sending plan to execute");

  task.introspection().publishSolution(*task.solutions().front());

  auto solutions = task.solutions();
  std::vector<decltype(solutions)::value_type> solutionsVector(solutions.begin(), solutions.end());
  std::sort(solutionsVector.begin(), solutionsVector.end(), std::greater<>());

  if (!solutionsVector.empty()) {
    auto plan_result = task.execute(*solutionsVector[0]);
    if (plan_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Task execution failed");
      return false;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Task execution succeeded");
    task.clear();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "No solutions found");
    task.clear();
    return false;
  }  
}

void ExecuteMoveToPredefined(
  const std::shared_ptr<GoalHandleMoveToPredefined> goal_handle,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner)
{
  RCLCPP_INFO(node->get_logger(), "Executing goal");

  auto result = std::make_shared<MoveToPredefined::Result>();
  result->success = false;
  auto goal = goal_handle->get_goal();

  auto task = ConfigureTask("move_to_predefined_task", node);

  RCLCPP_INFO(node->get_logger(), "Setting group: %s", goal->group_name.c_str());
  RCLCPP_INFO(node->get_logger(), "Setting goal: %s", goal->goal_pose.c_str());

  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(
    "current");
  task.add(std::move(stage_state_current));

  auto stage_move_to_predefined =
    std::make_unique<moveit::task_constructor::stages::MoveTo>(
    "predefined",
    interpolation_planner);
  stage_move_to_predefined->setGroup(goal->group_name);
  stage_move_to_predefined->setGoal(goal->goal_pose);
  task.add(std::move(stage_move_to_predefined));

  if (!SendTask(task, node))
  {
    result->success = false;   
    goal_handle->succeed(result);
    return;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Goal succeeded");
    result->success = true;
    goal_handle->succeed(result);
  }
}

void ExecutePick(
  const std::shared_ptr<GoalHandlePick> goal_handle,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner,
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Executing pick");
  
  auto result = std::make_shared<Pick::Result>();
  auto goal = goal_handle->get_goal();
  auto object = goal->object_goal;

  auto task_ = ConfigureTask("pick_task", node);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(goal->object_goal);

  moveit::task_constructor::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(
    "current");
  current_state_ptr = stage_state_current.get();
  task_.add(std::move(stage_state_current));
  
  auto stage_open_gripper =
    std::make_unique<moveit::task_constructor::stages::MoveTo>(
    "open_gripper",
    interpolation_planner);
  stage_open_gripper->setGroup(goal_handle->get_goal()->gripper_group);
  stage_open_gripper->setGoal(goal_handle->get_goal()->open_pose);  
  task_.add(std::move(stage_open_gripper));

  auto stage_move_to_pick = std::make_unique<moveit::task_constructor::stages::Connect>(
      "move to pick",
      moveit::task_constructor::stages::Connect::GroupPlannerVector{ { "arm_torso", interpolation_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(3.0);
  stage_move_to_pick->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
  task_.add(std::move(stage_move_to_pick));

  // clang-format off
  moveit::task_constructor::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on
  {
    auto grasp = std::make_unique<moveit::task_constructor::SerialContainer>("pick object");
    task_.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format off
    {
      auto stage =
          std::make_unique<moveit::task_constructor::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", "gripper_grasping_frame");
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "gripper_grasping_frame";
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object.id);
      stage->setAngleDelta(M_PI / 16);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame THIS IS SO IMPORTANT
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.0; //check this
      grasp_frame_transform.translation().x() = 0.03; //check this also this can be done like in a fallback

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<moveit::task_constructor::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, "gripper_grasping_frame");
      wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(object.id,
                             task_.getRobotModel()
                                 ->getJointModelGroup("gripper")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
      
    }

    {
      auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup("gripper");
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object.id, "gripper_grasping_frame");
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<moveit::task_constructor::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame("gripper_grasping_frame");
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 0.9;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task_.add(std::move(grasp));
  }
  {
    auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("clear scene");
    stage->removeObject(object.id);
    task_.add(std::move(stage));
  }

  if (!SendTask(task_, node))
  {
    result->success = false;   
    goal_handle->succeed(result);
    return;
  }
  else
  {
    if (EvaluateJoint(std::map<std::string, double>{ { "gripper_right_finger_joint", 0.0 }, { "gripper_left_finger_joint", 0.0 }}, 0.01, node))
    {
      RCLCPP_INFO(node->get_logger(), "Goal succeeded");
      result->success = true;
      goal_handle->succeed(result);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Goal failed");
      result->success = false;
      goal_handle->succeed(result);
    }
  } 

}

bool EvaluateJoint(const std::map<std::string, double>& desired_joint_values, const double& tolerance, const rclcpp::Node::SharedPtr& node)
{
  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  if (client->wait_for_service(2.0s)) {
		auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
		req->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;

    auto res_future = client->async_send_request(req);

		if (rclcpp::spin_until_future_complete(node, res_future) == rclcpp::FutureReturnCode::SUCCESS) {
			auto res = res_future.get();
			// check if names in desired_joint_values are in res->scene.robot_state.joint_state.name
    // Use std::all_of with a lambda function
    bool are_joints_values_within_tolerance = std::all_of(
      desired_joint_values.begin(),
      desired_joint_values.end(),
      [&res, tolerance](const auto& pair) {
          const std::string& joint_name = pair.first;
          double desired_value = pair.second;

          auto it = std::find(res->scene.robot_state.joint_state.name.begin(), res->scene.robot_state.joint_state.name.end(), joint_name);

          if (it != res->scene.robot_state.joint_state.name.end()) {
              // Joint name found, get the corresponding index
              size_t index = std::distance(res->scene.robot_state.joint_state.name.begin(), it);

              // Check if the value at the corresponding index is within the tolerance
              return std::abs(res->scene.robot_state.joint_state.position[index] - desired_value) <= tolerance;
          } else {
              // Joint name not found in robot state
              return false;
          }
      }
    );

		}
    else
    {
      return false;
    }                           
  }
  else
  {
    return false;
  }
}
} // end namespace manipulation
