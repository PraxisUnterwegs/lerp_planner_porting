#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_detailed_response.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include "lerp_interface/lerp_interface.h"

namespace lerp_interface
{
// 调用构造函数
LERPInterface::LERPInterface(const rclcpp::Node::SharedPtr& node) : node_(node), name_("LERPInterface")  // TODO
/*
arg nh:是一个节点对象引用，等效到moveit2里就是const rclcpp::Node::SharedPtr& node
: 后面是初始化列表
initializer[1]nh_的作用就是在创建这个变量的时候直接赋默认值了（初始化），nh_(nh)的意思就是 nh_ = const ros::NodeHandle& nh
initializer[2]name_("LERPInterface")就相当于 name_ = "LERPInterface"
initializer的成员变量的定义在头文件声明里就完成了
*/
{
}

bool LERPInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req,
                          moveit_msgs::msg::MotionPlanDetailedResponse& res)
{
  // Load the planner-specific parameters
  this->get_parameter("num_steps", num_steps_);  // moveit2里应该是this->get_parameter("num_steps", num_steps_); TODO
  // 查询参数"num_steps"，并且将查询结果存储到num_steps_成员变量中

  // 查询当前时间
  rclcpp::Clock clock;  // TODO
  rclcpp::Time start_time = clock.now();
  // 替换为ros2的api
  // rclcpp::Clock clock;  // 创建一个Clock对象
  // rclcpp::Time start_time = clock.now();  // 获取当前时间

  //从 PlanningScene 中获取机器人的模型。  
  const moveit::core::RobotModelConstPtr& robot_model = planning_scene.get_robot_model();
  // TODO
  // 从 PlanningScene 中获取机器人的模型。PlanningScene 类包含了机器人的模型、环境信息、碰撞检测状态等，它用于表示一个完整的规划场景。

  // 创建一个新的 RobotState 对象，并初始化它。输入参数就是前面获取的机器人模型。
  moveit::core::RobotStatePtr start_state = std::make_shared<moveit::core::RobotState>(robot_model);

  // 给 RobotState对象重新赋值
  *start_state = planning_scene->getCurrentState();

  // 获取指定的关节模型组（JointModelGroup）
  const moveit::core::JointModelGroup* joint_model_group = start_state->getJointModelGroup(req.group_name);

  // 获取关节模型组（JointModelGroup）中所有关节的名称
  std::vector<std::string> joint_names = joint_model_group->getVariableNames();

  // 获取关节组的总自由度；给自由度成员变量赋值
  dof_ = joint_names.size();

  // 将 RobotState 中当前关节模型组的关节值（例如关节的角度）复制到一个向量中
  std::vector<double> start_joint_values;
  // 读取对应关节的当前位置，并且将结果一一输出到数组
  start_state->copyJointGroupPositions(joint_model_group, start_joint_values);

  // This planner only supports one goal constraint in the request
  // 从请求中获取目标约束
  std::vector<moveit_msgs::Constraints> goal_constraints = req.goal_constraints;

  // 获取目标约束中的第一组约束（第一组约束是和关节约束相关的）
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = goal_constraints[0].joint_constraints;

  // 创建数组，用于存储目标关节的位置（即每个关节的目标角度、位置等
  std::vector<double> goal_joint_values;
  // 给goal_joint_values分配和goal_joint_constraint相同长度的位置（保证这俩数组的长度一致）
  goal_joint_values.reserve(goal_joint_constraint.size());  // reserve()用于预分配一定的内存空间来存储元素，而不是动态分配长度
  // 用于遍历 goal_joint_constraint 向量中的每个元素，将每个关节约束的目标位置（constraint.position）添加到 goal_joint_values 向量中。
  for (const auto& constraint : goal_joint_constraint)
  {
    goal_joint_values.push_back(constraint.position);
  }

  // ==================== Interpolation
  // 生成一个包含机器人关节轨迹的消息
  trajectory_msgs::msg::JointTrajectory joint_trajectory;  // TODO ROS2 api变化： trajectory_msgs::msg::JointTrajectory joint_trajectory; 

  // interpolate 是一个函数，它根据起始和目标关节位置，以及其他信息（如关节模型组和时间）来计算关节的运动轨迹。
  interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);
  // 参数都是上面刚刚定义的

  // ==================== feed the response
  // 将生成的关节轨迹（joint_trajectory）填充到一个响应消息（res）中
  // 将 res.trajectory 向量的大小设置为 1（只允许一个轨迹被保存）
  res.trajectory.resize(1);
  // 给消息res的成员属性赋值
  // 所有关节的名称
  res.trajectory[0].joint_trajectory.joint_names = joint_names;
  // 关节状态的时间戳和坐标系信息
  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;
  // 计算规划生成的轨迹
  res.trajectory[0].joint_trajectory = joint_trajectory;

  // 任务执行成功反馈
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  // 计算规划任务花费时间
  //res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());
  auto now = clock.now();
  double processing_time = (now - start_time).seconds();
  res.processing_time.push_back(processing_time);
  /* 
  换为ros2的api
  // 使用 rclcpp::Clock 获取当前时间
  rclcpp::Clock clock;
  auto now = clock.now();
  
  // 计算时间差并转换为秒
  double processing_time = (now - start_time).seconds();
  
  // 将处理时间推送到 processing_time 向量中
  res.processing_time.push_back(processing_time);
  */

  // 关节组名称
  res.group_name = req.group_name;
  // 所有关节名称
  res.trajectory_start.joint_state.name = joint_names;
  // 关节初始位置
  res.trajectory_start.joint_state.position = start_joint_values;

  return true;
}


// 插值函数的视线
void LERPInterface::interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& rob_state,
                                const moveit::core::JointModelGroup* joint_model_group,
                                const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                                trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
  // 调整 joint_trajectory.points 向量的大小。具体来说，它将 joint_trajectory.points 向量的大小设置为 num_steps_ + 1，这通常是为了在生成关节轨迹时为每个轨迹点分配空间。
  joint_trajectory.points.resize(num_steps_ + 1);  // 植树问题，轨迹点个数比规划步数多1个

  // 计算每个关节的运动步长（dt），并将其存储在 dt_vector 向量中，以便在运动规划过程中使用。
  std::vector<double> dt_vector;
  // 遍历每个关节计算运动步长
  for (int joint_index = 0; joint_index < dof_; ++joint_index)
  {
    // 等间距步长
    double dt = (goal_joint_vals[joint_index] - start_joint_vals[joint_index]) / num_steps_;
    dt_vector.push_back(dt);
  }

  // 遍历每个时间步
  for (int step = 0; step <= num_steps_; ++step)
  {
    std::vector<double> joint_values;
    // 遍历每个关节（每个自由度，因为一个关节有一个自由度）
    for (int k = 0; k < dof_; ++k)
    {
      // 线性插值算法：
          // 关节值 = 开始位置 + 步长次数 * 等间距步长
      double joint_value = start_joint_vals[k] + step * dt_vector[k];
      joint_values.push_back(joint_value);
    }
    // 将当前步骤的关节位置（joint_values）设置到 rob_state（机器人状态）中
    rob_state->setJointGroupPositions(joint_model_group, joint_values);
    // 更新机器人状态
    rob_state->update();

    // 往空的消息包里填充属性
    // 所有关节名称
    joint_trajectory.joint_names = joint_names;
    // 当前时间步的关节位置读取
    joint_trajectory.points[step].positions = joint_values;
  }
}

}  // namespace lerp_interface

/*
本源代码的结构编排参考：
## lerp_interface.cpp

### 求解函数`solve()`：

步骤：

1. 获取参数“num_steps"
2. 查询当前时间（开始）
3. 获取机器人模型
4. 获取机器人状态
5. 获取指定的关节模型组
6. 获取关节模型组中所有关节的名称
7. 获取关节组的总自由度
8. 读取关节组中所有关节当前的位置
9. 从请求消息类型中获取目标约束
10. 获取目标约束中的关节约束
11. 从关节约束中获取每一个关节对应的位置
12. 创建一个空的轨迹消息包，用来存放规划后的轨迹
13. 用插值函数生成轨迹(所有关节名称、RobotState对象、关节组模型、关节起始位置、关节目标位置、空的轨迹包)
14. 给`res`消息对象的属性赋值：
    1. 所有关节名称
    2. 关节状态的时间戳和坐标信息
    3. 空的轨迹消息包
    4. 规划花费时间
    5. 关节组名称
    6. 关节初始位置

---

### 插值函数`interpolate()`：

这部分其实很像定格动画，一步一步地推出动作。

步骤：

1. 计算每个关节的步长
2. 遍历时间步，依据算法计算每个时间步的结果位置
3. 更新每个时间步的机器人状态
4. 填充空白轨迹消息包的属性
*/
