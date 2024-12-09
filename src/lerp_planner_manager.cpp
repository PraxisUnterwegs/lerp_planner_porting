#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <class_loader/class_loader.hpp>
#include "lerp_interface/lerp_planning_context.h"
#include <pluginlib/class_list_macros.hpp>

namespace lerp_interface
{
// LERPPlannerManager类的实现，用于创建和管理LERP规划器。
class LERPPlannerManager : public planning_interface::PlannerManager
{
public:
  // 构造函数：调用基类PlannerManager的构造函数初始化。
  LERPPlannerManager() : planning_interface::PlannerManager()
  {
  }

  // 初始化函数：调用各个关节组（JointModelGroup）创建并存储对应的LERPPlanningContext实例
  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& /*ns*/) override
  {
     // 遍历机器人模型中的关节模型组名称
    for (const std::string& gpName : model->getJointModelGroupNames())
    {
       // 打印输出关节组和机器人模型的名称
      std::cout << "group name " << gpName << std::endl << "robot model  " << model->getName() << std::endl;
       // 使用字典容器 planning_contexts_ 存储 LERPPlanningContext 对象（初始化赋值）。这个字典容器是一个成员变量。
      planning_contexts_[gpName] = std::make_shared<LERPPlanningContext>("lerp_planning_context", gpName, model); // 键是 gpName（关节模型组名称）
    }
    return true;
  }

  //  检查请求是否可服务：当 req 中的轨迹约束为空时才能进行服务
  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  // 获取该规划器类的描述。
  std::string getDescription() const override
  {
    return "LERP";
  }

  // 返回可用的规划算法列表（现在只有单个算法"lerp"）（返回的是算法的名称）
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("lerp");
  }

  // 创建规划器上下文实例
  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;  // TODO msg api ros2 conversion

     // 检查是否指定了关节组，如果没有指定，会报错。并返回一个空的规划上下文指针，返回空指针表示无法处理当前的规划请求，因此不能继续执行规划任务。
    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

     // 检查是否提供了规划场景，如果没有指定，会报错。并返回一个空的规划上下文指针，返回空指针表示无法处理当前的规划请求，因此不能继续执行规划任务。
    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // create PlanningScene using hybrid collision detector
     // 创建一个新的不同的，PlanningScene对象
    planning_scene::PlanningScenePtr ps = planning_scene->diff();
     /*
     创建一个 差异化的副本，具体来说，diff() 会创建一个与当前 planning_scene 对象相比，只包含变化的部分 的新 PlanningScene 对象。
     这可以是环境、障碍物、机器人状态等的变化。
     差异化的副本可以提高效率，因为它只关注那些发生变化的部分，而不是每次都复制整个场景。
     */ 

    // set FCL as the allocaotor
     // 对规划场景对象PlanningScene对象设置碰撞检测。设置一个激活的碰撞检测器，并指定使用 FCL (Flexible Collision Library) 来执行碰撞检测
    ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create(), true);

    // retrieve and configure existing context
     // 获取指定关节组的上下文对象（即键值对的值）
    const LERPPlanningContextPtr& context = planning_contexts_.at(req.group_name);
    ROS_INFO_STREAM_NAMED("lerp_planner_manager", "===>>> context is made ");

     // 配置自定义的规划器上下文对象
    context->setPlanningScene(ps);  // 将当前的 规划场景 ps 设置到 context 对象中
    context->setMotionPlanRequest(req);  // 将 运动规划请求（req）设置到 context 对象中

     // 运行到这里都没有问题，就说明规划成功了
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    return context;
  }

protected:
   // 声明成员变量，是一个容器（键值对）
  std::map<std::string, LERPPlanningContextPtr> planning_contexts_;
  // 定义了一个 std::map 容器，用于存储 LERPPlanningContext 类型的指针，并以 std::string 类型的键进行索引
  // std::map 是 C++ 标准库中的一个 关联容器，它存储键值对（key-value pairs），并通过 键 来索引值。
  // 键是 gpName（关节模型组名称），值是一个 LERPPlanningContext 上下文对象。
};

}  // namespace lerp_interface

// register the LERPPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(lerp_interface::LERPPlannerManager, planning_interface::PlannerManager);
