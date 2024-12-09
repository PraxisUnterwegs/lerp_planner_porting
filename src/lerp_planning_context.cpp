#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit/planning_scene/planning_scene.h>

#include "lerp_interface/lerp_planning_context.h"
#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
// 构造函数初始化
LERPPlanningContext::LERPPlanningContext(const std::string& context_name, const std::string& group_name,
                                         const moveit::core::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
  lerp_interface_ = std::make_shared<LERPInterface>();  // TODO
}


// Context的solve方法重写实际上就是调用了interface的solve函数。这样把solve都封装在solve中。interface的solve中含有算法实现。
bool LERPPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  // 定义一个临时的响应消息对象
  moveit_msgs::msg::MotionPlanDetailedResponse res_msg;

  // 调用LERPInterface类的solve方法，执行规划。（算法实现部分）
  // 检测是否调用调用LERPInterface类的solve方法的solve函数并正常运行
  bool lerp_solved = lerp_interface_->solve(planning_scene_, request_, res_msg);  // 返回值被存储到了res_msg消息包对象中去了

  // 如果规划成功：
  if (lerp_solved)
  {
    // 将 res.trajectory 向量的大小设置为 1（只允许一个轨迹被保存）
    res.trajectory_.resize(1);
    // 创建了一个新的 RobotTrajectory 对象，存储机器人的规划轨迹
    res.trajectory_[0] = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());  // 每个规划组生成一个独立的轨迹。即，每个规划组有一个专门的轨迹规划

    
    // 将机器人状态从消息转换为RobotState对象。
    // 创建一个空的RobotState对象，准备承接来自消息res_msg的规划指示。
    moveit::core::RobotState start_state(robot_model_);
    moveit::core::robotStateMsgToRobotState(res_msg.trajectory_start, start_state);  // res_msg.trajectory_start 存储的是消息包中每个时间步的规划起点状态

    // 依据（每个时间步初始状态，消息包指示的每个时间步的规划结果）来计算轨迹，并保存到res参数的轨迹属性中
    res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_msg.trajectory[0]);
    // 规划阶段声明
    res.description_.push_back("plan");  
    // 计算规划时间
    res.processing_time_ = res_msg.processing_time;
    // 检测是否规划成功
    res.error_code_ = res_msg.error_code;

    return true;
  }

  // 如果调用solve规划的过程失败，就会报错
  res.error_code_ = res_msg.error_code;
  return false;
};

// 又重写了 context 方法，不过这次是简化版的，上面那个是详细版的
bool LERPPlanningContext::solve(planning_interface::MotionPlanResponse& res)
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



bool LERPPlanningContext::terminate()
{
  return true;
}

void LERPPlanningContext::clear()
{
  // This planner has no state, so has nothing to clear
}

}  // namespace lerp_interface
