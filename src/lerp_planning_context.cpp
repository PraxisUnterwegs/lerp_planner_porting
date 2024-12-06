#include <moveit/robot_state/conversions.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lerp_interface/lerp_planning_context.h"
#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
class LERPPlanningContext : public planning_interface::PlanningContext
{
public:
  LERPPlanningContext(const std::string& context_name, const std::string& group_name,
                      const moveit::core::RobotModelConstPtr& model)
    : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
  {
    lerp_interface_ = std::make_shared<LERPInterface>();
  }

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    moveit_msgs::msg::MotionPlanDetailedResponse res_msg;
    bool lerp_solved = lerp_interface_->solve(planning_scene_, request_, res_msg);

    if (lerp_solved)
    {
      res.trajectory_.resize(1);
      res.trajectory_[0] = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());

      moveit::core::RobotState start_state(robot_model_);
      moveit::core::robotStateMsgToRobotState(res_msg.trajectory_start, start_state);

      res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_msg.trajectory[0]);
      res.description_.push_back("plan");
      res.processing_time_ = res_msg.processing_time;
      res.error_code_ = res_msg.error_code;

      return true;
    }

    res.error_code_ = res_msg.error_code;
    return false;
  }

  bool solve(planning_interface::MotionPlanResponse& res) override
  {
    planning_interface::MotionPlanDetailedResponse res_detailed;
    bool planning_success = solve(res_detailed);

    res.error_code_ = res_detailed.error_code_;

    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ = res_detailed.processing_time_[0];
    }

    return planning_success;
  }

  bool terminate() override
  {
    return true;
  }

  void clear() override
  {
    // This planner has no state to clear
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::shared_ptr<LERPInterface> lerp_interface_;
};

}  // namespace lerp_interface

