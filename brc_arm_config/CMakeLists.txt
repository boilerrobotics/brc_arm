cmake_minimum_required(VERSION 3.22)
project(brc_arm_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

ament_package()

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input rclcpp geometry_msgs control_msgs)

install(TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
