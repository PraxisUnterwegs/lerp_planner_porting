
#pragma once

#include <rclcpp/rclcpp.hpp>  // 引入 ROS 的 C++ API，提供创建节点、发布和订阅消息、服务、定时器等功能。
#include <moveit/planning_interface/planning_interface.h>  // 引入基类支持

namespace lerp_interface
{
MOVEIT_CLASS_FORWARD(LERPInterface);  // 是 MoveIt! 中的一个宏，它用于声明一个类的前向声明（forward declaration）。
//这句目的在于在头文件声明类的时候，因为类的实现可能没写（纯虚函类），这样声明的时候就不能创建指针。但是如果有了这个宏，就能在纯虚类中创建指针了。

class LERPInterface  // 创建与namespace同名的类
{
public:
  LERPInterface(const rclcpp::Node::SharedPtr& node);  // 构造函数，参数是ros1的节点对象类，换到ros2中就是const rclcpp::Node::SharedPtr& node

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req, moveit_msgs::msg::MotionPlanDetailedResponse& res); // TODO moveit_msgs::msg::MotionPlanDetailedResponse& res
  /*
  arg[1]planning_scene:指向 PlanningScene 对象的常量共享指针。PlanningScene 是一个表示当前机器人的状态、环境和约束的对象，它包含了机器人的模型、碰撞检测信息、传感器数据等。（是个指针引用）
  arg[2]req:表示运动规划请求，包含了规划的目标、约束、起始状态、规划方式等信息。（是个类引用，换句话说就是类）
  arg[3]:输出详细的运动规划结果，包括路径、成功标志和其他相关数据。（是个类引用）
  */

protected:
  rclcpp::Node::SharedPtr node_;  // 节点对象
  std::string name_;  // 节点名称
  int num_steps_; // 运动规划的步数
  int dof_;  // 机械臂自由度的维度数

private:
  void interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& robot_state,
                   const moveit::core::JointModelGroup* joint_model_group, const std::vector<double>& start_joint_vals,
                   const std::vector<double>& goal_joint_vals, trajectory_msgs::msg::JointTrajectory& joint_trajectory);
  /*
  arg[1]joint_names:存储了机械臂（或其他机器人）关节的名称列表。这个参数提供了关节的名称，通常是与机械臂或机器人模型相关的关节名称。（是个数组引用）
  arg[2]robot_state:是 MoveIt! 中表示机器人当前状态的对象，包含机器人的关节位置、关节速度、关节加速度等信息。（是个指针引用）
  arg[3]joint_model_group:是 MoveIt! 中的一个对象，表示机器人关节模型的一个组。提供与该关节组相关的模型信息，确保在插值过程中正确操作特定的关节组。（是个指针对象）
  arg[4]start_joint_vals:中的每个元素表示一个关节的初始角度（或位置），通常是在插值开始时机器人的关节状态。（是个数组引用）
  arg[5]goal_joint_vals:包含了机器人各个关节的目标位置（目标关节值）。（数组引用）
  arg[6]joint_trajectory:表示一个关节轨迹, JointTrajectory 消息包含多个时间步，每个时间步对应机器人的关节位置、速度和加速度。它通常用于描述机器人关节在规划过程中的运动路径。（类引用）
  */
};
}  // namespace lerp_interface
