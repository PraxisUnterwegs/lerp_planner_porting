#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_detailed_response.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
class LERPInterface : public rclcpp::Node
{
public:
  LERPInterface(const std::string& node_name)
    : Node(node_name), name_("LERPInterface")
  {
    this->declare_parameter<int>("num_steps", 10);  // Default value
    RCLCPP_INFO(this->get_logger(), "LERPInterface node initialized.");
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req,
             moveit_msgs::msg::MotionPlanDetailedResponse& res)
  {
    // Load the planner-specific parameters
    this->get_parameter("num_steps", num_steps_);

    auto start_time = this->now();
    auto robot_model = planning_scene->getRobotModel();
    auto start_state = std::make_shared<moveit::core::RobotState>(robot_model);
    *start_state = planning_scene->getCurrentState();
    const auto* joint_model_group = start_state->getJointModelGroup(req.group_name);
    auto joint_names = joint_model_group->getVariableNames();
    dof_ = joint_names.size();

    std::vector<double> start_joint_values;
    start_state->copyJointGroupPositions(joint_model_group, start_joint_values);

    // Support for single goal constraint
    const auto& goal_constraints = req.goal_constraints;
    const auto& goal_joint_constraints = goal_constraints[0].joint_constraints;

    std::vector<double> goal_joint_values;
    for (const auto& constraint : goal_joint_constraints)
    {
      goal_joint_values.push_back(constraint.position);
    }

    // Interpolation
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);

    // Feed the response
    res.trajectory.resize(1);
    res.trajectory[0].joint_trajectory.joint_names = joint_names;
    res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;
    res.trajectory[0].joint_trajectory = joint_trajectory;

    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    res.processing_time.push_back((this->now() - start_time).seconds());

    res.group_name = req.group_name;
    res.trajectory_start.joint_state.name = joint_names;
    res.trajectory_start.joint_state.position = start_joint_values;

    return true;
  }

private:
  void interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& rob_state,
                   const moveit::core::JointModelGroup* joint_model_group,
                   const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                   trajectory_msgs::msg::JointTrajectory& joint_trajectory)
  {
    joint_trajectory.points.resize(num_steps_ + 1);

    std::vector<double> dt_vector;
    for (int joint_index = 0; joint_index < dof_; ++joint_index)
    {
      double dt = (goal_joint_vals[joint_index] - start_joint_vals[joint_index]) / num_steps_;
      dt_vector.push_back(dt);
    }

    for (int step = 0; step <= num_steps_; ++step)
    {
      std::vector<double> joint_values;
      for (int k = 0; k < dof_; ++k)
      {
        double joint_value = start_joint_vals[k] + step * dt_vector[k];
        joint_values.push_back(joint_value);
      }
      rob_state->setJointGroupPositions(joint_model_group, joint_values);
      rob_state->update();

      joint_trajectory.joint_names = joint_names;
      joint_trajectory.points[step].positions = joint_values;
    }
  }

  std::string name_;
  int num_steps_;
  int dof_;
};
}  // namespace lerp_interface


