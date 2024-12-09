cmake_minimum_required(VERSION 3.22)
project(moveit_lerp_planner)
# project(lerp_motion_planner)

# dependency
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(moveit_visual_tools REQUIRED)


# Contains header file directories
include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(${moveit_visual_tools_INCLUDE_DIRS})
include_directories(/home/jzf/ws_moveit/install/moveit_visual_tools/include)


# Add an executable file
add_executable(lerp_example src/lerp_example.cpp)
ament_target_dependencies(lerp_example rclcpp Boost moveit_core moveit_ros_planning moveit_msgs)

# Add a link library
target_link_libraries(lerp_example ${Boost_LIBRARIES})

# Add a library of plugins
add_library(moveit_lerp_planner_plugin
  src/lerp_planner_manager.cpp
  src/lerp_interface.cpp
  src/lerp_planning_context.cpp)
set_target_properties(moveit_lerp_planner_plugin PROPERTIES VERSION "${PROJECT_VERSION}")
ament_target_dependencies(moveit_lerp_planner_plugin rclcpp pluginlib Boost moveit_core moveit_ros_planning moveit_msgs)



# INSTALL
install(
  TARGETS
    lerp_example
    moveit_lerp_planner_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(
  FILES lerp_interface_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/doc)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp pluginlib Boost moveit_core moveit_ros_planning moveit_msgs)


# Export the plug-in description file
pluginlib_export_plugin_description_file(moveit_core lerp_interface_plugin_description.xml)


ament_package()

