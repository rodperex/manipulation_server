#include "manipulation/manipulation_behaviors.hpp"

namespace manipulation
{

using namespace std::chrono_literals;

moveit::task_constructor::Task configure_task(
  const std::string & task_name,
  rclcpp::Node::SharedPtr node)
{

  moveit::task_constructor::Task task;

  task.stages()->setName(task_name);
  task.loadRobotModel(node);

  task.setProperty("ik_frame", node->get_parameter("ik_frame").as_string());
  task.setProperty("group", node->get_parameter("group").as_string());
  task.setProperty("eef", node->get_parameter("eef").as_string());

  RCLCPP_INFO(node->get_logger(), "Task %s configured", task_name.c_str());

  return task;
}

bool execute_task(moveit::task_constructor::Task & task, rclcpp::Node::SharedPtr node)
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
    // task.clear();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(), "No solutions found");
    // task.clear();
    return false;
  }
}

moveit::task_constructor::Task move_to_predefined_task(
  std::string group_name,
  std::string goal_pose,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner)
{
  RCLCPP_INFO(node->get_logger(), "Executing goal");

  auto task = configure_task("move_to_predefined_task", node);

  {
    auto stage = std::make_unique<
      moveit::task_constructor::stages::CurrentState>(
      "current");
    task.add(std::move(stage));
  }

  auto stage_move_to_predefined =
    std::make_unique<moveit::task_constructor::stages::MoveTo>(
    "predefined",
    interpolation_planner);
  stage_move_to_predefined->setGroup(group_name);
  stage_move_to_predefined->setGoal(goal_pose);
  task.add(std::move(stage_move_to_predefined));

  return task;
}

moveit::task_constructor::Task move_joint_task(
  std::string group_name,
  std::string joint_name,
  double joint_value,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner)
{
  RCLCPP_INFO(node->get_logger(), "Executing goal");

  auto task = configure_task("move_joint_task", node);

  {
    auto stage = std::make_unique<
      moveit::task_constructor::stages::CurrentState>(
      "current");
    task.add(std::move(stage));
  }
  {
    auto stage =
      std::make_unique<moveit::task_constructor::stages::MoveTo>(
        "move_joint",
        interpolation_planner);
    RCLCPP_INFO(node->get_logger(), "Setting group: %s", node->get_parameter("group").as_string().c_str());
    // stage->setGroup(node->get_parameter("group").as_string());
    stage->setGroup(group_name);
    
    RCLCPP_INFO(node->get_logger(), "Setting goal: %s to %f", joint_name.c_str(), joint_value);
    stage->setGoal(std::map<std::string, double>{{joint_name, joint_value}});

    task.add(std::move(stage));
  }

  return task;
}

moveit::task_constructor::Task move_end_effector_task(
  geometry_msgs::msg::PoseStamped pose,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner)
{
  RCLCPP_INFO(node->get_logger(), "Executing goal");

  auto task = configure_task("move_eef_task", node);

  auto cartesian = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
	cartesian->setJumpThreshold(2.0);

  const auto ptp = [&node]() {
		auto pp{ std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node, "pilz_industrial_motion_planner") };
		pp->setPlannerId("PTP");
		return pp;
	}();

  const auto rrtconnect = [&node]() {
		auto pp{ std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node, "ompl") };
		pp->setPlannerId("RRTConnect");
		return pp;
	}();

  {
    auto stage = std::make_unique<
      moveit::task_constructor::stages::CurrentState>(
      "current");
    task.add(std::move(stage));
  }

  auto fallbacks = std::make_unique<moveit::task_constructor::Fallbacks>("posible_solutions");
  auto add_to_fallbacks{ [&](auto& solver, auto& name) {
		auto move_to = std::make_unique<moveit::task_constructor::stages::MoveTo>(name, solver);
    move_to->setIKFrame(node->get_parameter("ik_frame").as_string());
		move_to->setGroup(node->get_parameter("arm_group").as_string());
		move_to->setGoal(pose);
		fallbacks->add(std::move(move_to));
	} };

  add_to_fallbacks(cartesian, "Cartesian path");
  add_to_fallbacks(interpolation_planner, "Interpolation path");
	add_to_fallbacks(ptp, "PTP path");
	add_to_fallbacks(rrtconnect, "RRT path");


  task.add(std::move(fallbacks));

  return task;
}

moveit::task_constructor::Task pick_task(
  moveit_msgs::msg::CollisionObject object,
  moveit::task_constructor::Stage * & attach_object_stage,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner,
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner,
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Executing pick");

  auto result = std::make_shared<Pick::Result>();

  auto task = configure_task("pick_task", node);

  psi->applyCollisionObject(object);

  moveit::task_constructor::Stage * current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(
    "current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  std::string arm_group =
    node->get_parameter("arm_group").as_string();
  std::string gripper_group =
    node->get_parameter("gripper_group").as_string();
  std::string open_pose =
    node->get_parameter("open_pose").as_string();
  std::string close_pose =
    node->get_parameter("close_pose").as_string();

  // 1. Open gripper
  RCLCPP_INFO(node->get_logger(), "1.- Open gripper");
  auto stage_open_gripper =
    std::make_unique<moveit::task_constructor::stages::MoveTo>(
    "open_gripper",
    interpolation_planner);

  stage_open_gripper->setGroup(gripper_group);
  stage_open_gripper->setGoal(open_pose);
  task.add(std::move(stage_open_gripper));


  // 2. Move to pick
  RCLCPP_INFO(node->get_logger(), "2.- Move to pick");
  auto stage_move_to_pick = std::make_unique<moveit::task_constructor::stages::Connect>(
    "move_to_pick",
    moveit::task_constructor::stages::Connect::GroupPlannerVector{
      {arm_group, interpolation_planner}
    });

  stage_move_to_pick->setTimeout(3.0);
  stage_move_to_pick->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));
  RCLCPP_INFO(node->get_logger(), "Move to pick stage added");

  // 3. Pick object
  RCLCPP_INFO(node->get_logger(), "3.- Pick object");
  {
    auto grasp = std::make_unique<moveit::task_constructor::SerialContainer>("pick_object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    // clang-format off
    grasp->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT,
      {"eef", "group", "ik_frame"});

    {
      // 3.1. Approach object
      RCLCPP_INFO(node->get_logger(), "\t3.1.- Approach object");
      auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        "approach_object",
        cartesian_planner);

      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", node->get_parameter("ik_frame"));
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.0, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = (node->get_parameter("ik_frame")).as_string();
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    {
      // 3.2. Generate grasp pose
      RCLCPP_INFO(node->get_logger(), "\t3.2.- Generate grasp pose");

      auto grasp_poses = std::make_unique<moveit::task_constructor::Alternatives>("grasp_poses");
      task.properties().exposeTo(grasp_poses->properties(), {"eef", "group", "ik_frame"});
      grasp_poses->properties().configureInitFrom(
        moveit::task_constructor::Stage::PARENT,
        {"eef", "group", "ik_frame"});    

      {
        // asumes object is a cylinder: [height, radious] 

        // we define 5 iterations for the object height:
        auto botton_object_corner = -object.primitives[0].dimensions[0] / 2;

        auto height_iterations = 3;
        auto height_increment = object.primitives[0].dimensions[0] / height_iterations;

        for (int i = 0; i < height_iterations; ++i) {
          auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>(
          "generate_grasp_pose");
          stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
          stage->properties().set("marker_ns", "grasp_pose");
          stage->setPreGraspPose(open_pose);
          stage->setObject(object.id);
          stage->setAngleDelta(M_PI / 3);
          stage->setMonitoredStage(current_state_ptr);  // Hook into current state

          // This is the transform from the object frame to the end-effector frame THIS IS SO IMPORTANT
          Eigen::Isometry3d grasp_frame_transform;
          Eigen::Quaterniond q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
          grasp_frame_transform.linear() = q.matrix();
          //TO DO: adapt to object shape
          grasp_frame_transform.translation().z() = botton_object_corner + ( i * height_increment );
          grasp_frame_transform.translation().x() = 0.03; //check this also this can be done like in a fallback

          // Compute IK
          auto wrapper =
            std::make_unique<moveit::task_constructor::stages::ComputeIK>(
            "grasp_pose_IK",
            std::move(stage));
          wrapper->setMaxIKSolutions(8); // param?
          wrapper->setMinSolutionDistance(1.0); // param?
          wrapper->setIKFrame(grasp_frame_transform, (node->get_parameter("ik_frame")).as_string());
          wrapper->properties().configureInitFrom(
            moveit::task_constructor::Stage::PARENT, {"eef",
              "group"});
          wrapper->properties().configureInitFrom(
            moveit::task_constructor::Stage::INTERFACE,
            {"target_pose"});
          // grasp->insert(std::move(wrapper));
          grasp_poses->insert(std::move(wrapper));
        }
        auto stage = std::make_unique<moveit::task_constructor::stages::GenerateGraspPose>(
        "generate_grasp_pose_top");
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose(open_pose);
        stage->setObject(object.id);
        stage->setAngleDelta(M_PI / 3);
        stage->setMonitoredStage(current_state_ptr);  // Hook into current state

        // This is the transform from the object frame to the end-effector frame THIS IS SO IMPORTANT
        Eigen::Isometry3d grasp_frame_transform;
        Eigen::Quaterniond q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
        grasp_frame_transform.linear() = q.matrix();
        //TO DO: adapt to object shape
        grasp_frame_transform.translation().x() = botton_object_corner*-1 + 0.03;

        // Compute IK
        auto wrapper =
          std::make_unique<moveit::task_constructor::stages::ComputeIK>(
          "grasp_pose_IK",
          std::move(stage));
        wrapper->setMaxIKSolutions(8); // param?
        wrapper->setMinSolutionDistance(1.0); // param?
        wrapper->setIKFrame(grasp_frame_transform, (node->get_parameter("ik_frame")).as_string());
        wrapper->properties().configureInitFrom(
          moveit::task_constructor::Stage::PARENT, {"eef",
            "group"});
        wrapper->properties().configureInitFrom(
          moveit::task_constructor::Stage::INTERFACE,
          {"target_pose"});
          grasp_poses->insert(std::move(wrapper));

      }
      grasp->insert(std::move(grasp_poses));
    }

    {
      // 3.3. Allow collisions with the object so it can be grasped
      RCLCPP_INFO(node->get_logger(), "\t3.3.- Allow collisions");
      auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("allow_collision");
      stage->allowCollisions(
        object.id,
        task.getRobotModel()
        ->getJointModelGroup(node->get_parameter("eef").as_string())
        ->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));

    }

    {
      // 3.4. Close gripper
      RCLCPP_INFO(node->get_logger(), "\t3.4.- Close gripper");
      auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>(
        "close_hand",
        interpolation_planner);
      stage->setGroup(node->get_parameter("eef").as_string());
      stage->setGoal(close_pose);
      grasp->insert(std::move(stage));
    }

    {
      // 3.5. Attach object
      RCLCPP_INFO(node->get_logger(), "\t3.5.- Attach object");
      auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
        "attach_object");
      stage->attachObject(object.id, node->get_parameter("ik_frame").as_string());
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // 3.6. Lift object
      RCLCPP_INFO(node->get_logger(), "\t3.6.- Lift object");
      auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        "lift_object",
        cartesian_planner);
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.0, 0.3);
      stage->setIKFrame(node->get_parameter("ik_frame").as_string());
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 0.9;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    RCLCPP_INFO(node->get_logger(), "pick added");
    task.add(std::move(grasp));
  }

  return task;
}

void PlaceAfterPickTask(
  moveit_msgs::msg::CollisionObject object,
  geometry_msgs::msg::PoseStamped place_pose,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner,
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner,
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> sampling_planner,
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi,
  moveit::task_constructor::Stage * attach_object_stage,
  moveit::task_constructor::Task & task)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Executing place");

  std::string arm_group =
    node->get_parameter("arm_group").as_string();
  std::string gripper_group =
    node->get_parameter("gripper_group").as_string();
  std::string open_pose =
    node->get_parameter("open_pose").as_string();
  std::string close_pose =
    node->get_parameter("close_pose").as_string();

  auto stage_move_to_place = std::make_unique<moveit::task_constructor::stages::Connect>(
    "move_to_place",
    moveit::task_constructor::stages::Connect::GroupPlannerVector{
      {arm_group, sampling_planner},
      {gripper_group, sampling_planner}
    });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
  {
    auto place = std::make_unique<moveit::task_constructor::SerialContainer>("place_object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT,
      {"eef", "group", "ik_frame"});
    {
      // Stage to generate the poses to place the object and compute the IK
      auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePlacePose>(
        "generate place pose");
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object.id);

      stage->setPose(place_pose);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage. This allows
                                                      // the stage to know how the object is attached

      // Compute IK
      auto wrapper =
        std::make_unique<moveit::task_constructor::stages::ComputeIK>(
        "place_pose_IK",
        std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(object.id);
      wrapper->properties().configureInitFrom(
        moveit::task_constructor::Stage::PARENT, {"eef",
          "group"});
      wrapper->properties().configureInitFrom(
        moveit::task_constructor::Stage::INTERFACE,
        {"target_pose"});
      place->insert(std::move(wrapper));
    }
    {
      // Stage to detach the object from the hand
      auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>(
        open_pose,
        interpolation_planner);
      stage->setGroup(gripper_group);
      stage->setGoal(open_pose);
      place->insert(std::move(stage));
    }
    {
      // We no longer need to hold the object
      auto stage =
        std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>("forbid_collision");
      stage->allowCollisions(
        object.id,
        task.getRobotModel()
        ->getJointModelGroup(gripper_group)
        ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(stage));
    }
    {
      // Stage to detach the object from the hand
      auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
        "detach_object");
      stage->detachObject(object.id, node->get_parameter("ik_frame").as_string());
      place->insert(std::move(stage));
    }
    {
      // Stage to retreat the hand from the object
      auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        "retreat",
        cartesian_planner);
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.0, 0.15);
      stage->setIKFrame(node->get_parameter("ik_frame").as_string());
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = node->get_parameter("ik_frame").as_string();
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

}

moveit::task_constructor::Task pick_and_place_task(
  moveit_msgs::msg::CollisionObject object,
  geometry_msgs::msg::PoseStamped place_pose,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner,
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner,
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> sampling_planner,
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi)
{
  moveit::task_constructor::Stage * attach_object_stage_ptr;

  auto task = pick_task(
    object,
    attach_object_stage_ptr,
    node,
    interpolation_planner,
    cartesian_planner,
    psi);

  PlaceAfterPickTask(
    object,
    place_pose,
    node,
    interpolation_planner,
    cartesian_planner,
    sampling_planner,
    psi,
    attach_object_stage_ptr,
    task);

  return task;
}

moveit::task_constructor::Task place_task(
  moveit_msgs::msg::CollisionObject object,
  geometry_msgs::msg::PoseStamped place_pose,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner,
  std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> cartesian_planner,
  std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> sampling_planner,
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi)
{
  moveit::task_constructor::Task task;
  moveit::task_constructor::Stage * attach_object_stage_ptr;

  task = configure_task("place_task", node);

  moveit::task_constructor::Stage * current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(
    "current");

  task.add(std::move(stage_state_current));

  auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
    "attach_object");
  stage->attachObject(object.id, node->get_parameter("ik_frame").as_string());
  attach_object_stage_ptr = stage.get();
  task.add(std::move(stage));

  PlaceAfterPickTask(
    object,
    place_pose,
    node,
    interpolation_planner,
    cartesian_planner,
    sampling_planner,
    psi,
    attach_object_stage_ptr,
    task);

  return task;

}

moveit::task_constructor::Task detach_object_task(
  moveit_msgs::msg::CollisionObject object,
  rclcpp::Node::SharedPtr node)
{
  moveit::task_constructor::Task task;

  task = configure_task("detach_object_task", node);

  moveit::task_constructor::Stage * current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(
    "current");

  task.add(std::move(stage_state_current));

  auto stage = std::make_unique<moveit::task_constructor::stages::ModifyPlanningScene>(
    "detach_object");
  stage->detachObject(object.id, node->get_parameter("ik_frame").as_string());
  task.add(std::move(stage));

  return task;
}

moveit::task_constructor::Task pick_from_pc_task(
  sensor_msgs::msg::PointCloud2 object,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> interpolation_planner)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Executing pick from pc");

  std::string retreat_axis, config_file, gripper_group, gripper_joint;
  std::vector<std::string> gripper_joints;
  std::vector<double> gripper_min_limits, gripper_max_limits;
  int axis_index;
  float retreat_min, retreat_max, retreat_resolution;
  std::vector<std::tuple<float, float, float>> pc_points_xyz;

  // node->get_parameter("pick_from_pc.retreat_axis", retreat_axis);
  // node->get_parameter("pick_from_pc.retreat_min", retreat_min);
  // node->get_parameter("pick_from_pc.retreat_max", retreat_max);
  // node->get_parameter("pick_from_pc.retreat_resolution", retreat_resolution);
  node->get_parameter("pick_from_pc.config_file", config_file);
  node->get_parameter("gripper_group", gripper_group);
  node->get_parameter("gripper_joints", gripper_joints);
  node->get_parameter("gripper_closed", gripper_min_limits);
  node->get_parameter("gripper_open", gripper_max_limits);

  gpd::GraspDetector* grasp_detector = new gpd::GraspDetector(config_file);
  RCLCPP_INFO_STREAM(node->get_logger(), "Grasp detector created");

  auto cloud_camera = process_point_cloud(object);

  grasp_detector->preprocessPointCloud(*cloud_camera);

  auto candidates = grasp_detector->detectGrasps(*cloud_camera);
  RCLCPP_INFO_STREAM(node->get_logger(), "Grasping candidates detected");


  auto task = configure_task("pick_from_pc_task", node);

  RCLCPP_INFO_STREAM(node->get_logger(), "Task initialized");


  // {
  //   auto stage = std::make_unique<
  //     moveit::task_constructor::stages::CurrentState>(
  //     "current");
  //   task.add(std::move(stage));
  // }

  RCLCPP_INFO_STREAM(node->get_logger(), "Current state added");

  auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
	cartesian_planner->setJumpThreshold(2.0);

  const auto ptp = [&node]() {
		auto pp{ std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node, "pilz_industrial_motion_planner") };
		pp->setPlannerId("PTP");
		return pp;
	}();

  const auto rrtconnect = [&node]() {
		auto pp{ std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node, "ompl") };
		pp->setPlannerId("RRTConnect");
		return pp;
	}();

  const auto interpolation = [&node]() {
    auto pp{std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>()};
    return pp;
  }();

  auto fallbacks = std::make_unique<moveit::task_constructor::Fallbacks>("posible_solutions");

  auto add_to_fallbacks{ [&](auto& solver, auto& name, auto& pose, auto& width) {
    // auto serial = std::make_unique<moveit::task_constructor::SerialContainer>("move_nd_close");
		auto move_to = std::make_unique<moveit::task_constructor::stages::MoveTo>(name, solver);
    // auto close_gripper = std::make_unique<moveit::task_constructor::stages::MoveTo>("close_gripper", solver);

    // close_gripper->setGroup(gripper_group);  
    // close_gripper->setGoal(std::map<std::string, double>{{gripper_joints[0], gripper_min_limits[0]},
                                                  // {gripper_joints[1], width}});

		move_to->setGroup(node->get_parameter("arm_group").as_string());

    // Eigen::Isometry3d grasp_frame_transform;
    // //TO DO: adapt to object shape
    // grasp_frame_transform.translation().x() = pose.pose.position.x;
    // grasp_frame_transform.translation().y() = pose.pose.position.y;
    // grasp_frame_transform.translation().z() = pose.pose.position.z;
    // grasp_frame_transform.linear() = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z).matrix();

    move_to->setIKFrame(node->get_parameter("ik_frame").as_string());
		move_to->setGoal(pose);

    // serial->insert(std::move(move_to));
    // RCLCPP_INFO_STREAM(node->get_logger(), "Move to added");
    // serial->insert(std::move(close_gripper));
    RCLCPP_INFO_STREAM(node->get_logger(), "Close gripper added");

		fallbacks->add(std::move(move_to));
	} };

  if (candidates.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "No grasping candidates found");
    return task;
  }
  // for (const auto& grasp : candidates)
  // {
  //   geometry_msgs::msg::PoseStamped pose;
    
  //   // auto grasp = std::move(candidates[0]);
  //   pose.header.frame_id = object.header.frame_id;

  //   pose.pose.position.x = grasp->getPosition().x();
  //   pose.pose.position.y = grasp->getPosition().y();
  //   pose.pose.position.z = grasp->getPosition().z();

  //   //print header frame id
  //   RCLCPP_INFO_STREAM(node->get_logger(), "Header frame id: " << pose.header.frame_id);
  //   RCLCPP_INFO_STREAM(node->get_logger(), "Grasp position XYZ: " << grasp->getPosition().x() << " " << grasp->getPosition().y() << " " << grasp->getPosition().z());

  //   auto orientation_matrix = grasp->getOrientation();
  //   Eigen::Quaterniond q(orientation_matrix);

  //   pose.pose.orientation.x = q.x();
  //   pose.pose.orientation.y = q.y();
  //   pose.pose.orientation.z = q.z();
  //   pose.pose.orientation.w = q.w();
  //   RCLCPP_INFO_STREAM(node->get_logger(), "Grasp orientation XYZW: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w());
  //   // 0.069, 0.969, -0.229, 0.061
  //   // pose.pose.orientation.w = 1.0;

  //   double width = grasp->getGraspWidth();

  //   add_to_fallbacks(cartesian, "Cartesian path", pose, width);
  //   add_to_fallbacks(interpolation, "Interpolation path", pose, width);
  //   add_to_fallbacks(ptp, "PTP path", pose, width);
  //   add_to_fallbacks(rrtconnect, "RRT path", pose, width);
  //   RCLCPP_INFO_STREAM(node->get_logger(), "one candidate added");
  // }

  // task.add(std::move(fallbacks));
  // RCLCPP_INFO_STREAM(node->get_logger(), "Fallbacks added");

  // return task;


 ///// COMPLETE PICK TASK :

  RCLCPP_INFO_STREAM(node->get_logger(), "Executing pick");
  // psi->applyCollisionObject(object);

  moveit::task_constructor::Stage * current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>(
    "current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  std::string arm_group =
    node->get_parameter("arm_group").as_string();
  // std::string gripper_group =
  //   node->get_parameter("gripper_group").as_string();
  std::string open_pose =
    node->get_parameter("open_pose").as_string();
  std::string close_pose =
    node->get_parameter("close_pose").as_string();

  // 1. Open gripper
  RCLCPP_INFO(node->get_logger(), "1.- Open gripper");
  auto stage_open_gripper =
    std::make_unique<moveit::task_constructor::stages::MoveTo>(
    "open_gripper",
    interpolation_planner);

  stage_open_gripper->setGroup(gripper_group);
  stage_open_gripper->setGoal(open_pose);
  task.add(std::move(stage_open_gripper));


  // 2. Move to pick
  RCLCPP_INFO(node->get_logger(), "2.- Move to pick");
  auto stage_move_to_pick = std::make_unique<moveit::task_constructor::stages::Connect>(
    "move_to_pick",
    moveit::task_constructor::stages::Connect::GroupPlannerVector{
      {arm_group, interpolation_planner}
    });

  stage_move_to_pick->setTimeout(3.0);
  stage_move_to_pick->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
  stage_move_to_pick->setMaxDistance(0.1);
  task.add(std::move(stage_move_to_pick));
  RCLCPP_INFO(node->get_logger(), "Move to pick stage added");

  // 3. Pick object
  RCLCPP_INFO(node->get_logger(), "3.- Pick object");
  {
    auto grasp = std::make_unique<moveit::task_constructor::SerialContainer>("pick_object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    // clang-format off
    grasp->properties().configureInitFrom(
      moveit::task_constructor::Stage::PARENT,
      {"eef", "group", "ik_frame"});

    {
      // 3.1. Approach object
      RCLCPP_INFO(node->get_logger(), "\t3.1.- Approach object");
      auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        "approach_object",
        cartesian_planner);

      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", node->get_parameter("ik_frame"));
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.0, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = (node->get_parameter("ik_frame")).as_string();
      vec.vector.x = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    {
      // 3.2. Generate grasp pose
      RCLCPP_INFO(node->get_logger(), "\t3.2.- Generate grasp pose");

      auto grasp_poses = std::make_unique<moveit::task_constructor::Alternatives>("grasp_poses");
      task.properties().exposeTo(grasp_poses->properties(), {"eef", "group", "ik_frame"});
      grasp_poses->properties().configureInitFrom(
        moveit::task_constructor::Stage::PARENT,
        {"eef", "group", "ik_frame"});    

      {

        for (const auto& grasp : candidates)
        {
          geometry_msgs::msg::PoseStamped pose;
          auto stage = std::make_unique<moveit::task_constructor::stages::GeneratePose>(
          "generate_grasp_pose");
           stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT);
          stage->properties().set("marker_ns", "grasp_pose");
          // stage->setPreGraspPose(open_pose);
          // stage->setObject(object.id);
          // stage->setAngleDelta(M_PI / 3);
          stage->setMonitoredStage(current_state_ptr);  

          
          // auto grasp = std::move(candidates[0]);
          pose.header.frame_id = object.header.frame_id;

          pose.pose.position.x = grasp->getPosition().x();
          pose.pose.position.y = grasp->getPosition().y();
          pose.pose.position.z = grasp->getPosition().z();

          //print header frame id
          RCLCPP_INFO_STREAM(node->get_logger(), "Header frame id: " << pose.header.frame_id);
          RCLCPP_INFO_STREAM(node->get_logger(), "Grasp position XYZ: " << grasp->getPosition().x() << " " << grasp->getPosition().y() << " " << grasp->getPosition().z());

          auto orientation_matrix = grasp->getOrientation();
          Eigen::Quaterniond q(orientation_matrix);

          pose.pose.orientation.x = q.x();
          pose.pose.orientation.y = q.y();
          pose.pose.orientation.z = q.z();
          pose.pose.orientation.w = q.w();
          RCLCPP_INFO_STREAM(node->get_logger(), "Grasp orientation XYZW: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w());

          stage->setPose(pose);
          
          // Compute IK
          auto wrapper =
            std::make_unique<moveit::task_constructor::stages::ComputeIK>(
            "grasp_pose_IK",
            std::move(stage));
          wrapper->setMaxIKSolutions(8); // param?
          wrapper->setMinSolutionDistance(1.0); // param?
          wrapper->setIKFrame((node->get_parameter("ik_frame")).as_string());
          wrapper->properties().configureInitFrom(
            moveit::task_constructor::Stage::PARENT, {"eef",
              "group"});
          wrapper->properties().configureInitFrom(
            moveit::task_constructor::Stage::INTERFACE,
            {"target_pose"});
          grasp_poses->insert(std::move(wrapper));
  
        }

      }
      grasp->insert(std::move(grasp_poses));
    }

    {
      // 3.3. Close gripper
      RCLCPP_INFO(node->get_logger(), "\t3.3.- Close gripper");
      auto stage = std::make_unique<moveit::task_constructor::stages::MoveTo>(
        "close_hand",
        interpolation_planner);
      stage->setGroup(node->get_parameter("eef").as_string());
      stage->setGoal(close_pose);
      grasp->insert(std::move(stage));
    }

    {
      // 3.6. Lift object
      RCLCPP_INFO(node->get_logger(), "\t3.6.- Lift object");
      auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveRelative>(
        "lift_object",
        cartesian_planner);
      stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.0, 0.3);
      stage->setIKFrame(node->get_parameter("ik_frame").as_string());
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 0.9;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    RCLCPP_INFO(node->get_logger(), "pick added");
    task.add(std::move(grasp));
  }

  return task;

}

std::vector<geometry_msgs::msg::PoseStamped>  generate_grasp_poses(
  sensor_msgs::msg::PointCloud2 object,
  rclcpp::Node::SharedPtr node)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Executing generating grasp poses from pc");

  auto poses = std::vector<geometry_msgs::msg::PoseStamped>();

  std::string retreat_axis, config_file, gripper_group, gripper_joint;
  std::vector<std::string> gripper_joints;
  std::vector<double> gripper_min_limits, gripper_max_limits;
  int axis_index;
  float retreat_min, retreat_max, retreat_resolution;
  std::vector<std::tuple<float, float, float>> pc_points_xyz;

  // node->get_parameter("pick_from_pc.retreat_axis", retreat_axis);
  // node->get_parameter("pick_from_pc.retreat_min", retreat_min);
  // node->get_parameter("pick_from_pc.retreat_max", retreat_max);
  // node->get_parameter("pick_from_pc.retreat_resolution", retreat_resolution);
  node->get_parameter("pick_from_pc.config_file", config_file);
  node->get_parameter("gripper_group", gripper_group);
  node->get_parameter("gripper_joints", gripper_joints);
  node->get_parameter("gripper_closed", gripper_min_limits);
  node->get_parameter("gripper_open", gripper_max_limits);

  gpd::GraspDetector* grasp_detector = new gpd::GraspDetector(config_file);
  RCLCPP_INFO_STREAM(node->get_logger(), "Grasp detector created");

  auto cloud_camera = process_point_cloud(object);

  grasp_detector->preprocessPointCloud(*cloud_camera);

  auto candidates = grasp_detector->detectGrasps(*cloud_camera);
  RCLCPP_INFO_STREAM(node->get_logger(), "Grasping candidates detected");

  if (candidates.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "No grasping candidates found");
    return poses;
  }
  for (const auto& grasp : candidates)
  {
    geometry_msgs::msg::PoseStamped pose;
    
    pose.header.frame_id = object.header.frame_id;

    pose.pose.position.x = grasp->getPosition().x();
    pose.pose.position.y = grasp->getPosition().y();
    pose.pose.position.z = grasp->getPosition().z();
 
    //print header frame id
    RCLCPP_INFO_STREAM(node->get_logger(), "Header frame id: " << pose.header.frame_id);
    RCLCPP_INFO_STREAM(node->get_logger(), "Grasp position XYZ: " << grasp->getPosition().x() << " " << grasp->getPosition().y() << " " << grasp->getPosition().z());

    auto orientation_matrix = grasp->getOrientation();
    Eigen::Quaterniond q(orientation_matrix);

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    RCLCPP_INFO_STREAM(node->get_logger(), "Grasp orientation XYZW: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w());

    poses.push_back(pose);
  }
  return poses;
}


bool IsGripperClosed(rclcpp::Node::SharedPtr node)
{
  std::map<std::string, double> desired_joint_values;
  std::vector<std::string> gripper_joints;
  std::vector<double> gripper_tolerances, gripper_closed;

  node->get_parameter("gripper_joints", gripper_joints);
  node->get_parameter("gripper_tolerances", gripper_tolerances);
  node->get_parameter("gripper_closed", gripper_closed);

  if ((gripper_joints.size() != gripper_tolerances.size()) ||
    (gripper_joints.size() != gripper_closed.size()))
  {
    RCLCPP_INFO(node->get_logger(), "Impossible to get desired gripper joints values");
    return false;
  }
  for (size_t i = 0; i < gripper_joints.size(); i++) {
    desired_joint_values[gripper_joints[i]] = gripper_closed[i];
  }

  return evaluate_joint(desired_joint_values, gripper_tolerances);
}
bool evaluate_joint(
  const std::map<std::string, double> & desired_joint_values,
  const std::vector<double> & tolerances)
{
  // Not the best option creating a node here, but it is the easiest way
  auto node_aux = rclcpp::Node::make_shared("service_client_node");
  bool are_joints_values_within_tolerance = false;
  auto client = node_aux->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");

  if (client->wait_for_service(2.0s)) {
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;

    auto res_future = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(
        node_aux,
        res_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto res = res_future.get();
      // Check if names in desired_joint_values are in res->scene.robot_state.joint_state.name
      // Use std::all_of with a lambda function
      are_joints_values_within_tolerance = std::all_of(
        desired_joint_values.begin(),
        desired_joint_values.end(),
        [&res, tolerances](const auto & pair) {
          const std::string & joint_name = pair.first;
          double desired_value = pair.second;

          auto it =
          std::find(
            res->scene.robot_state.joint_state.name.begin(),
            res->scene.robot_state.joint_state.name.end(), joint_name);

          if (it != res->scene.robot_state.joint_state.name.end()) {
            // Joint name found, get the corresponding index
            size_t index = std::distance(res->scene.robot_state.joint_state.name.begin(), it);
            // Check if the value at the corresponding index is within the tolerance
            return std::abs(
              res->scene.robot_state.joint_state.position[index] - desired_value) <=
            tolerances[index];
          } else {
            // Joint name not found in robot state
            return false;
          }
        }
      );
    }
  }
  return are_joints_values_within_tolerance;
}

gpd::util::Cloud* process_point_cloud(sensor_msgs::msg::PointCloud2 pc)
{
  Eigen::Matrix3Xd view_points(3,1);
  view_points << 0.0, 0.0, 0.0;
  gpd::util::Cloud* cloud_camera;
  // matbe we have to set another view point rather than 0 0 0

  if (pc.fields.size() == 6 && pc.fields[3].name == "normal_x" && pc.fields[4].name == "normal_y" && pc.fields[5].name == "normal_z")
  {
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(pc, *cloud);
    cloud_camera = new gpd::util::Cloud(cloud, 0, view_points);

  }
  else
  {
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(pc, *cloud);
    cloud_camera = new gpd::util::Cloud(cloud, 0, view_points);
  }

  return cloud_camera;
}

} // end namespace manipulation
