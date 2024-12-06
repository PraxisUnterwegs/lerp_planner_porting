#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.hpp>
#include <moveit/utils/logger.hpp>
#include "lerp_interface/lerp_planning_context.h"

#include <pluginlib/class_list_macros.hpp>

namespace lerp_interface
{
class LERPPlannerManager : public planning_interface::PlannerManager
{
public:
  LERPPlannerManager() : planning_interface::PlannerManager()
  {
    RCLCPP_INFO(getLogger(), "LERPPlannerManager initialized.");
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    node_ = node;
    for (const std::string& group_name : model->getJointModelGroupNames())
    {
      RCLCPP_INFO(getLogger(), "Initializing planning context for group '%s'", group_name.c_str());
      planning_contexts_[group_name] =
          std::make_shared<LERPPlanningContext>("lerp_planning_context", group_name, model);
    }
    return true;
  }

  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "LERP";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("lerp");
  }

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      RCLCPP_ERROR(getLogger(), "No group specified to plan for.");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return nullptr;
    }

    if (!planning_scene)
    {
      RCLCPP_ERROR(getLogger(), "No planning scene supplied as input.");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return nullptr;
    }

    // Create a diff PlanningScene with FCL collision detector
    planning_scene::PlanningScenePtr ps = planning_scene->diff();
    ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create(), true);

    // Retrieve and configure existing context
    const auto& context = planning_contexts_.at(req.group_name);
    RCLCPP_INFO(getLogger(), "Configuring planning context for group '%s'", req.group_name.c_str());

    context->setPlanningScene(ps);
    context->setMotionPlanRequest(req);

    return context;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, std::shared_ptr<LERPPlanningContext>> planning_contexts_;

  static rclcpp::Logger getLogger()
  {
    return rclcpp::get_logger("moveit.planners.lerp.planner_manager");
  }
};

}  // namespace lerp_interface

PLUGINLIB_EXPORT_CLASS(lerp_interface::LERPPlannerManager, planning_interface::PlannerManager)

