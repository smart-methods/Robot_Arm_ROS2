cmake_minimum_required(VERSION 3.22)
project(arduinobot_mc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch scripts
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()



install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/moveit_time_fix.py
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(time_param_node src/time_param_node.cpp)
ament_target_dependencies(time_param_node
  rclcpp
  moveit_ros_planning_interface
  moveit_core
)

install(TARGETS time_param_node
  DESTINATION lib/${PROJECT_NAME}
)
