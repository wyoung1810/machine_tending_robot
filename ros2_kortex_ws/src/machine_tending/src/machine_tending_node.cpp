#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <vector>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("machine_tending");
namespace mtc = moveit::task_constructor;

class MachineTendingNode : public rclcpp::Node
{
public:
  MachineTendingNode(const rclcpp::NodeOptions& options)
  // MachineTendingNode()
  : Node("machine_tending_node", options)
  {
    // Make arduino_to_arm subscriber
    arduino_to_arm_sub_ = this->create_subscription<std_msgs::msg::String>(
    "arduino_to_arm", 10, std::bind(&MachineTendingNode::topic_callback, this, std::placeholders::_1));

    // Make arm_to_relay publisher
    arm_to_relay_pub_ = this->create_publisher<std_msgs::msg::String>("arm_to_relay", 10);
  }

  void doTask(std::function<mtc::Task()> func, std::string arm_state_);

  void setupPlanningScene();
  void originalPlanningScene();

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Process the received message
    RCLCPP_INFO(LOGGER, "Received: %s", msg->data.c_str());

    if (msg->data == "load") {
      RCLCPP_INFO(LOGGER, "Load");
      // this->doTask([this](){return this->openHand();}, "load");
      this->doTask([this](){return this->loadStock();}, "load");
      // this->doTask([this](){return this->createTask();}, "load");
    }
    if (msg->data == "unload") {
      RCLCPP_INFO(LOGGER, "Unload");
      this->doTask([this](){return this->closeHand();}, "unload");
    }
  }
  void arm_pub(std::string arm_state_)
  {
    auto message_ = std_msgs::msg::String();
    message_.data = arm_state_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
    arm_to_relay_pub_->publish(message_);
  }
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task openHand();
  mtc::Task closeHand();
  mtc::Task loadStock();
  mtc::Task task_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arduino_to_arm_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_to_relay_pub_;
};

moveit_msgs::msg::CollisionObject makeBox(float x_size_, float y_size_, float z_size_, float x_pos_, float y_pos_, float z_pos_, std::string id_)
{
  // Make box to avoid
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "world";
  box.id = id_;
  shape_msgs::msg::SolidPrimitive box_box;

  // Define the size of the box in meters
  box_box.type = box_box.BOX;
  box_box.dimensions.resize(3);
  box_box.dimensions[box_box.BOX_X] = x_size_;
  box_box.dimensions[box_box.BOX_Y] = y_size_;
  box_box.dimensions[box_box.BOX_Z] = z_size_;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
  box_pose.position.x = x_pos_;
  box_pose.position.y = y_pos_;
  box_pose.position.z = z_pos_;

  box.primitives.push_back(box_box);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  return box;
}

void MachineTendingNode::originalPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void MachineTendingNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // Make object to pick
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  shape_msgs::msg::SolidPrimitive object_box;

  // Define the size of the box in meters
  object_box.type = object_box.BOX;
  object_box.dimensions.resize(3);
  object_box.dimensions[object_box.BOX_X] = 0.01;
  object_box.dimensions[object_box.BOX_Y] = 0.04; // longest edge
  object_box.dimensions[object_box.BOX_Z] = 0.04;

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5; // m
  pose.position.y = 0.25; // m
  pose.position.z = 0.02; // m
  pose.orientation.w = 0.612;
  pose.orientation.x = -0.354;
  pose.orientation.y = 0.354;
  pose.orientation.z = 0.612;
  
  object.primitives.push_back(object_box);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;
  psi.applyCollisionObject(object);

  // Make pick base
  moveit_msgs::msg::CollisionObject pick_base;
  pick_base.id = "pick_base";
  pick_base.header.frame_id = "world";
  shape_msgs::msg::SolidPrimitive pick_base_box;

  // Define the size of the box in meters
  pick_base_box.type = pick_base_box.BOX;
  pick_base_box.dimensions.resize(3);
  pick_base_box.dimensions[pick_base_box.BOX_X] = 0.15;
  pick_base_box.dimensions[pick_base_box.BOX_Y] = 0.1;
  pick_base_box.dimensions[pick_base_box.BOX_Z] = 0.01;

  geometry_msgs::msg::Pose pick_base_pose;
  pick_base_pose.position.x = 0.5; // m
  pick_base_pose.position.y = 0.25; // m
  pick_base_pose.position.z = -0.005; // m
  pick_base_pose.orientation.w = 0.683;
  pick_base_pose.orientation.x = 0.183;
  pick_base_pose.orientation.y = -0.183;
  pick_base_pose.orientation.z = 0.683;
  
  pick_base.primitives.push_back(pick_base_box);
  pick_base.primitive_poses.push_back(pick_base_pose);
  pick_base.operation = pick_base.ADD;
  psi.applyCollisionObject(pick_base);

  // Make table to avoid
  moveit_msgs::msg::CollisionObject table = makeBox(2.0, 2.0, 0.1, 0.0, 0.0, -0.055, "table");
  psi.applyCollisionObject(table);

  // Make wall to avoid tormach side wall 
  moveit_msgs::msg::CollisionObject tormach_side_wall = makeBox(0.1, 2.0, 0.8, -0.15, 0.0, 0.4, "sidewall");
  psi.applyCollisionObject(tormach_side_wall);

  // Make ceiling to avoid
  moveit_msgs::msg::CollisionObject tormach_ceiling = makeBox(1.0, 0.75, 0.1, 0.25, -0.5, 0.75, "ceiling");
  psi.applyCollisionObject(tormach_ceiling);

  // Make wall to avoid tormach back wall
  moveit_msgs::msg::CollisionObject tormach_back = makeBox(1.0, 0.1, 1.0, 0.25, -0.6, 0.5, "back");
  psi.applyCollisionObject(tormach_back);

  // Make block to avoid spindle
  moveit_msgs::msg::CollisionObject spindle = makeBox(0.2, 0.5, 0.7, 0.6, -0.6, 0.55, "spindle");
  psi.applyCollisionObject(spindle);
}

void MachineTendingNode::doTask(std::function<mtc::Task()> func, std::string arm_state_)
{
  task_ = func();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task init failed: " << e);
    // RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  arm_pub(arm_state_);

  return;
}

mtc::Task MachineTendingNode::openHand()
{
  mtc::Task task_;
  task_.stages()->setName("pick and place");
  task_.loadRobotModel(shared_from_this());

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "robotiq_85_base_link";

  // Set task properties
  task_.setProperty("group", arm_group_name);
  task_.setProperty("eef", hand_group_name);
  task_.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task_.add(std::move(stage_state_current));

  // Define planers and reconfigure their action servers
  const double scaling_factor = 0.1;
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
  sampling_planner->setMaxVelocityScalingFactor(scaling_factor);
  sampling_planner->setMaxAccelerationScalingFactor(scaling_factor);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(scaling_factor);
  interpolation_planner->setMaxAccelerationScalingFactor(scaling_factor);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(scaling_factor);
  cartesian_planner->setMaxAccelerationScalingFactor(scaling_factor);
  cartesian_planner->setStepSize(.01);

  // auto stage_open_hand =
  //     std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // stage_open_hand->setGroup(hand_group_name);
  // stage_open_hand->setGoal("Open");
  // task_.add(std::move(stage_open_hand));

  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("Home");
    task_.add(std::move(stage));

  return task_;
}

mtc::Task MachineTendingNode::closeHand()
{
  mtc::Task task_;
  task_.stages()->setName("pick and place");
  task_.loadRobotModel(shared_from_this());

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "robotiq_85_base_link";

  // Set task properties
  task_.setProperty("group", arm_group_name);
  task_.setProperty("eef", hand_group_name);
  task_.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task_.add(std::move(stage_state_current));

  // Define planers and reconfigure their action servers
  const double scaling_factor = 0.1;
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(scaling_factor);
  interpolation_planner->setMaxAccelerationScalingFactor(scaling_factor);

  auto stage_close_hand =
      std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage_close_hand->setGroup(hand_group_name);
  stage_close_hand->setGoal("Close");
  task_.add(std::move(stage_close_hand));

  return task_;
}

mtc::Task MachineTendingNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("pick and place");
  task.loadRobotModel(shared_from_this());
  task.setProperty("velocity_scaling", 0.1);

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "robotiq_85_base_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // Define planners and reconfigure their action servers
  const double scaling_factor = 0.1;
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
  sampling_planner->setMaxVelocityScalingFactor(scaling_factor);
  sampling_planner->setMaxAccelerationScalingFactor(scaling_factor);
  sampling_planner->setPlannerId("RRTConnect_scaled");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(scaling_factor);
  interpolation_planner->setMaxAccelerationScalingFactor(scaling_factor);
  // interpolation_planner->setPlannerId("RRTConnect_scaled");

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(scaling_factor);
  cartesian_planner->setMaxAccelerationScalingFactor(scaling_factor);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("Open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("Open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.13;

      // Compute IK
      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");
    
      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");
    
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.y = 0.1;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
    
      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");
    
      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("Home");
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task MachineTendingNode::loadStock()
{
  mtc::Task task;
  task.stages()->setName("load stock");
  task.loadRobotModel(shared_from_this());
  task.setProperty("velocity_scaling", 0.1);

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "robotiq_85_base_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  // Define planers and reconfigure their action servers
  const double scaling_factor = 0.4;
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
  sampling_planner->setMaxVelocityScalingFactor(scaling_factor);
  sampling_planner->setMaxAccelerationScalingFactor(scaling_factor);
  sampling_planner->setPlannerId("RRTConnect_scaled");

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(scaling_factor);
  interpolation_planner->setMaxAccelerationScalingFactor(scaling_factor);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(scaling_factor);
  cartesian_planner->setMaxAccelerationScalingFactor(scaling_factor);
  cartesian_planner->setStepSize(.01);

  auto stage_home = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  stage_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_home->setGoal("Home");
  task.add(std::move(stage_home));

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("Open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("Open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.145; // Distance away from target

      // Compute IK
      auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");
    
      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");
    
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = 0.38;
      target_pose_msg.pose.position.y = -0.44;
      target_pose_msg.pose.position.z = 0.2;
      target_pose_msg.pose.orientation.w = 0.707;
      target_pose_msg.pose.orientation.x = 0.0;
      target_pose_msg.pose.orientation.y = 0.707;
      target_pose_msg.pose.orientation.z = 0.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
    
      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }
    

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("place in vise", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.005, 0.12);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "place_in_vise");
    
      // Set downward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("Open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }
    
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("move up", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "move_up");
    
      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("move out of tormach", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.6);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "move_out");
    
      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.y = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  auto last_home = std::make_unique<mtc::stages::MoveTo>("last home", interpolation_planner);
  last_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  last_home->setGoal("Home");
  task.add(std::move(last_home));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto machine_tending_task_node = std::make_shared<MachineTendingNode>(options);
  machine_tending_task_node->setupPlanningScene();
  // machine_tending_task_node->originalPlanningScene();

  rclcpp::spin(machine_tending_task_node);

  rclcpp::shutdown();
  return 0;
}