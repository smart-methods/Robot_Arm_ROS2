cmake_minimum_required(VERSION 3.5)
project(arduinobot_description)


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY launch meshes models urdf rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  PROGRAMS
    scripts/send_joint_angles.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/joint_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)