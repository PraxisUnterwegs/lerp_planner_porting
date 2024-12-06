#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include "lerp_interface/lerp_interface.hpp"

namespace lerp_interface
{
MOVEIT_CLASS_FORWARD(LERPPlanningContext);

class LERPPlanningContext : public planning_interface::PlanningContext
{
public:
  LERPPlanningContext(const std::string& name, const std::string& group, const moveit::core::RobotModelConstPtr& model);
  ~LERPPlanningContext() override = default;

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;
  void clear() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  LERPInterfacePtr lerp_interface_;
};

}  // namespace lerp_interface
