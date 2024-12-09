#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
//#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  //const std::string NODE_NAME = "lerp_example";
  //ros::init(argc, argv, NODE_NAME);
  //ros::AsyncSpinner spinner(1);
  //spinner.start();
  //ros::NodeHandle node_handle("~");
  const std::string NODE_NAME = "lerp_example";
  rclcpp::init(argc, argv);  // ROS 2 使用 rclcpp::init 初始化节点
  // 创建节点对象
  auto node = rclcpp::Node::make_shared(NODE_NAME);
   // 这上面的一块为ros1的api，需要修改为ros2的：
   /*
   // 初始化 ROS 2 节点
    const std::string NODE_NAME = "lerp_example";
    rclcpp::init(argc, argv);  // ROS 2 使用 rclcpp::init 初始化节点

    // 创建节点对象
    auto node = rclcpp::Node::make_shared(NODE_NAME);

    末尾别忘了：
    rclcpp::executors::MultiThreadedExecutor spinner;
    spinner.add_node(node);
    spinner.spin();  // 处理回调，多个线程并行执行
    // 节点使用结束时清理资源
    rclcpp::shutdown();
   */

  // 指定关节组
  const std::string PLANNING_GROUP = "panda_arm";
  // 指定参数（参数服务器），robot_description 用于指定 urdf 文件（机器人模型）
  const std::string ROBOT_DESCRIPTION = "robot_description";  // TODO,这个参数，ros1里直接由参数服务器发布，但是ros2里必须手动在launch文件里创建
  // 加载机器人模型（根据参数 "robot_description"）
  //robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));
  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, ROBOT_DESCRIPTION);
  // Create a planning scene monitor
  // 创建规划场景监视器
  //planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, robot_model_loader);
  // 监控和维护机器人的 规划场景（PlanningScene），它确保规划场景中的环境和机器人的状态信息是最新的。
  // 换句话说，监视器就是实现实时更新的东西。场景监视器自然是实现场景实时更新的工具

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();
  
  // 获取机器人模型
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // Create a RobotState and to keep track of the current robot pose and planning group
 // 创建 RobotState 对象并设置默认值
  //moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm);
  if (locked_scene)
    {
        // 初始化 RobotState
        auto robot_state = std::make_shared<moveit::core::RobotState>(locked_scene->getCurrentState());
        RCLCPP_INFO(node->get_logger(), "Robot state successfully initialized.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to lock PlanningScene.");
    }

  robot_state->setToDefaultValues();
  robot_state->update();

  // Create JointModelGroup
  // 获取关节模型组
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& link_model_names = joint_model_group->getLinkModelNames();
  // 获取末端执行器名称
  RCLCPP_INFO(node->get_logger(), "end effector name %s", link_model_names.back().c_str());
  // .back() 方法返回 link_model_names 中的最后一个元素
  // .c_str() 方法将 std::string 转换为 C 风格的字符串

  // Set the planner
  // 将规划器插件名称设置为参数，并发布到 ROS 参数服务器 中，以便其他节点或组件能够访问该参数。
  std::string planner_plugin_name = "lerp_interface/LERPPlanner";
  //node_handle.setParam("planning_plugin", planner_plugin_name); // TODO，api老旧
  node->set_parameter(rclcpp::Parameter("planning_plugin", planner_plugin_name));
  // ROS2的api：node->set_parameter(rclcpp::Parameter("planning_plugin", planner_plugin_name));

  // Create pipeline
  //planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));  // TODO
  auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "planning_plugin", "request_adapters");
  // api老旧
  // 参数request_adapters在launch文件里加载的yaml文件中进行定义和发布
  // request_adapters 是一个包含多个适配器插件的列表，这些适配器会在规划请求发送到规划器之前对请求进行修改或调整。

  // ================================ Set the start and goal joint state
  // 消息对象实例化
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = PLANNING_GROUP;

  // Get the joint values of the start state and set them in request.start_state
  // 各个关节起始值
  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  // Goal constraint
  // 关节目标值（手动设定）
  std::vector<double> goal_joint_values = { 0.8, 0.7, 1, 1.3, 1.9, 2.2, 3 };
  robot_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  robot_state->update();
  moveit_msgs::msg::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  req.goal_constraints[0].name = "goal_pos";

  // Set joint tolerance
  // 设置关节允许的公差
  std::vector<moveit_msgs::msg::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
  for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
  {
    //ROS_INFO_STREAM_NAMED(NODE_NAME, " ======================================= joint position at goal: "<< goal_joint_constraint[x].position);
    RCLCPP_INFO_STREAM(node->get_logger(), " ======================================= joint position at goal: " << goal_joint_constraint[x].position);
    req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
    req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
  }

  // ================================ Visualization tools
  // 可视化工具
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, psm);
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();
  visual_tools.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  //
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  visual_tools.trigger();

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // ================================ planning context
  // 创建一个局部作用域，约束变量的存在周期
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    // 生成规划（example源文件的核心一步）
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    //ROS_ERROR_STREAM_NAMED(NODE_NAME, "Could not compute plan successfully");
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not compute plan successfully");
    
    return 0;
  }

  visual_tools.prompt("Press 'next' to visualzie the result");

  // ================================ Visualize the trajectory
  //  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.trigger();

  // 可视化轨迹
  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/display_planned_path", 1, true);
  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1, true);
  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  moveit_msgs::msg::MotionPlanResponse response; // TODO
  res.getMessage(response);

  
  moveit_msgs::msg::RobotTrajectory solution_traj = response.trajectory;
  int number_of_steps = solution_traj.joint_trajectory.points.size();
  // ROS_DEBUG_NAMED(NODE_NAME, "number of timesteps in the solution trajectory: %i", number_of_steps);
  RCLCPP_DEBUG(node->get_logger(), "number of timesteps in the solution trajectory: %d", number_of_steps);

  for (int step_num = 0; step_num < number_of_steps; ++step_num)
  {
    std::vector<double> solution_positions;
    solution_positions = solution_traj.joint_trajectory.points[step_num].positions;
    std::stringstream sst;
    for (double solution_position : solution_positions)
    {
      sst << solution_position << " ";
    }
    //ROS_INFO_STREAM_NAMED(NODE_NAME, sst.str());
    RCLCPP_INFO(node->get_logger(), "%s", sst.str().c_str());
  }

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);  //  返回 display_trajectory.trajectory 中的最后一个轨迹（即刚刚添加的规划轨迹）。
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  //display_publisher.publish(display_trajectory);
  display_publisher->publish(display_trajectory);

  /* We can also use visual_tools to wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  
  // 处理回调，多个线程并行执行
  rclcpp::executors::MultiThreadedExecutor spinner;
  spinner.add_node(node);
  spinner.spin();  
  // 节点使用结束时清理资源
  rclcpp::shutdown();
}

/*
本源代码的结构编排参考：

## lerp_example.cpp

### 主函数 `main`：

#### 初始化：
1. 初始化 ROS 节点、异步 spinner。
2. 加载机器人模型和规划场景监视器。

#### 设置规划器：
1. 指定 LERPPlanner 插件。
2. 配置规划管道。

#### 设置起始和目标状态：
1. 从当前机器人状态获取起始关节值。
2. 手动设置目标关节值并构造目标约束。
3. 设置关节容差。

#### 使用可视化工具：
1. 清理 RViz 的标记。
2. 显示规划信息。
3. 等待用户输入。

#### 执行规划：
1. 使用 `PlanningPipeline` 生成规划。
2. 检查规划结果。
3. 将规划结果发布到 RViz。
*/
