#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lerp_example");

  RCLCPP_INFO(node->get_logger(), "Starting LERP Example Node");

  const std::string PLANNING_GROUP = "panda_arm";
  const std::string ROBOT_DESCRIPTION = "robot_description";

  // Load robot model
  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, ROBOT_DESCRIPTION);
  auto robot_model = robot_model_loader->getModel();

  // Create Planning Scene Monitor
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, robot_model_loader);

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  auto robot_state = std::make_shared<moveit::core::RobotState>(psm->getPlanningScene()->getCurrentState());
  robot_state->setToDefaultValues();
  robot_state->update();

  // Create JointModelGroup
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  RCLCPP_INFO(node->get_logger(), "End effector name: %s", link_model_names.back().c_str());

  // Set the planner plugin
  node->declare_parameter<std::string>("planning_plugin", "lerp_interface/LERPPlanner");
  std::string planner_plugin_name = node->get_parameter("planning_plugin").as_string();

  // Create Planning Pipeline
  auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
      robot_model, psm->getPlanningScene(), node, "planning_plugin", "request_adapters");

  // Define Motion Plan Request and Response
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = PLANNING_GROUP;

  // Set start state
  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  // Define goal state
  std::vector<double> goal_joint_values = {0.8, 0.7, 1, 1.3, 1.9, 2.2, 3};
  robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  robot_state->update();

  auto joint_goal = kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
  req.goal_constraints.push_back(joint_goal);
  req.goal_constraints[0].name = "goal_pos";

  // Set joint tolerances
  for (auto& joint_constraint : req.goal_constraints[0].joint_constraints)
  {
    joint_constraint.tolerance_above = 0.001;
    joint_constraint.tolerance_below = 0.001;
  }

  // Visualization tools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", rvt::RVIZ_MARKER_TOPIC, psm);
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.publishText(Eigen::Isometry3d::Identity(), "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to start the demo");

  // Planning
  {
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm);
    planning_pipeline->generatePlan(locked_scene, req, res);
  }

  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not compute plan successfully");
    rclcpp::shutdown();
    return 0;
  }

  visual_tools.prompt("Press 'next' to visualize the result");

  // Visualize the trajectory
  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  res.getMessage(display_trajectory.trajectory_start, display_trajectory.trajectory.emplace_back());
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher->publish(display_trajectory);

  visual_tools.prompt("Press 'next' to continue the demo");

  rclcpp::shutdown();
  return 0;
}

