add_executable(test_load_mujoco_ros2_control test_load_mujoco_ros2_control.cpp)
target_include_directories(test_load_mujoco_ros2_control
                           PRIVATE ${MUJOCO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../include)
ament_target_dependencies(test_load_mujoco_ros2_control rclcpp rclcpp_lifecycle
                          hardware_interface pluginlib)
target_link_libraries(test_load_mujoco_ros2_control ${LIB_MUJOCO})

install(TARGETS test_load_mujoco_ros2_control DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS test_joint_trajectory_publisher.py
        DESTINATION lib/${PROJECT_NAME})
