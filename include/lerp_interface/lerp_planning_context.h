#ifndef LERP_PLANNING_CONTEXT_H
#define LERP_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
MOVEIT_CLASS_FORWARD(LERPPlanningContext);  // 允许LERPPlanningContext类在声明时即可使用指针

class LERPPlanningContext : public planning_interface::PlanningContext
{
public:
  LERPPlanningContext(const std::string& name, const std::string& group, const moveit::core::RobotModelConstPtr& model);
  /*
  arg[1]name:机器人的名称（变量引用）
  arg[2]group:机器人的运动组所属（变量引用）
  arg[3]model:MoveIt! 中用于表示机器人的模型类。该指针通常用于表示一个机器人模型，它提供有关机器人的结构、关节、连接等信息。（指针引用）
  */
  ~LERPPlanningContext() override  // 重写析构函数
  {
    
  }

  bool solve(planning_interface::MotionPlanResponse& res) override;  
  // arg res：是 MoveIt! 中的一个类，通常用于存储运动规划请求的响应结果。它包含了规划结果、状态、执行的轨迹、执行的时间等信息。（类引用）

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
  // arg res：是 MotionPlanResponse类的一个扩展，提供了更详细的响应信息。它可能额外包括其他详细的调试信息、计算的路径段、每个路径点的执行时间、各种成本评估等。（类引用）

  bool terminate() override;
  void clear() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;  // RobotModel类用于表示机器人模型。它包含有关机器人各个部分的信息，如关节、链条、连接、坐标系等。ConstPtr确保只读。
  moveit::core::RobotStatePtr robot_state_;  // RobotState 类用于表示机器人的当前状态。它包含机器人的关节位置、速度、加速度等信息。
  LERPInterfacePtr lerp_interface_;  // LERPInterface的实例对象LERPInterfacePtr，lerp_interface_是指针对象。
};

}  // namespace lerp_interface

#endif  // LERP_PLANNING_CONTEXT_H
