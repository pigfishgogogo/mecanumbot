cmake_minimum_required(VERSION 3.8)
project(mecanumbot_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED) #
find_package(pluginlib REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


# Add pluginlib export
add_definitions(-DPLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# Create CAN interface library
add_library(can_interface_lib SHARED
  src/can_interface.cpp
)
# Create motor controller library
add_library(motor_controller_lib SHARED
  src/motor_controller.cpp
)
ament_target_dependencies(
  motor_controller_lib
  "rclcpp"
  "std_msgs"
)
target_link_libraries(motor_controller_lib can_interface_lib)



# Create the main hardware interface library
add_library(mecanumbot_hardware_lib SHARED
  src/mecanumbot_hardware.cpp
)
# 新增：关联 controller_manager_msgs 等依赖（核心修复）
ament_target_dependencies(
  mecanumbot_hardware_lib
  "controller_interface"
  "controller_manager_msgs"
  "controller_manager"
  "hardware_interface"
  "rclcpp"
  "rclcpp_lifecycle"
)
target_link_libraries(mecanumbot_hardware_lib
  motor_controller_lib
  can_interface_lib
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${hardware_interface_LIBRARIES}
)

# Add plugin export
pluginlib_export_plugin_description_file(hardware_interface mecanumbot_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface mecanumbot_hardware.xml)

# Create system hardware interface
add_executable(mecanumbot_system
  src/mecanumbot_system.cpp
)
# 新增：关联 controller_manager_msgs 等依赖（核心修复，解决头文件查找失败）
ament_target_dependencies(
  mecanumbot_system
  "controller_interface"
  "controller_manager_msgs"
  "controller_manager"
  "hardware_interface"
  "rclcpp"
  "rclcpp_lifecycle"
)
target_link_libraries(mecanumbot_system
  mecanumbot_hardware_lib
  motor_controller_lib
  can_interface_lib
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${hardware_interface_LIBRARIES}
)


# Install libraries
install(TARGETS
  can_interface_lib
  motor_controller_lib
  mecanumbot_hardware_lib
  mecanumbot_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



# # ROS2 node
# add_executable(canopen_motor_node src/canopen_motor_node.cpp)
# target_link_libraries(canopen_motor_node
#   motor_controller_lib
#   can_interface_lib
#   ${rclcpp_LIBRARIES}
# )

# # Install targets
# install(TARGETS
#   test_motor_control
#   canopen_motor_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# Install URDF files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
endif()
# Install config files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# Install plugin description
install(FILES mecanumbot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()