add_executable(test_load_mujoco_ros2_control test_load_mujoco_ros2_control.cpp)
target_include_directories(test_load_mujoco_ros2_control
                           PRIVATE ${MUJOCO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../include)
ament_target_dependencies(test_load_mujoco_ros2_control rclcpp rclcpp_lifecycle
                          hardware_interface pluginlib)
target_link_libraries(test_load_mujoco_ros2_control ${LIB_MUJOCO})

install(TARGETS test_load_mujoco_ros2_control DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS test_joint_trajectory_publisher.py
                 test_scene_manager_services.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING AND MUJOCO_ROS_UTILS_BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(
    test_roadmap_fixtures
    test_roadmap_fixtures.py)

  ament_add_gtest(
    test_mujoco_plugin_loading
    test_mujoco_plugin_loading.cpp)
  if(TARGET test_mujoco_plugin_loading)
    target_include_directories(
      test_mujoco_plugin_loading
      PRIVATE ${MUJOCO_INCLUDE_DIR})
    ament_target_dependencies(test_mujoco_plugin_loading hardware_interface)
    target_link_libraries(test_mujoco_plugin_loading ${LIB_MUJOCO})
    target_compile_definitions(
      test_mujoco_plugin_loading
      PRIVATE
        MUJOCO_ROS_UTILS_TEST_XML_DIR="${PROJECT_SOURCE_DIR}/xml")

    set(plugin_test_environment
        "MUJOCO_ROS_UTILS_TEST_XML_DIR=${PROJECT_SOURCE_DIR}/xml")
    if(TARGET MujocoRosUtilsClockPublisherPlugin)
      list(
        APPEND plugin_test_environment
        "MUJOCO_ROS_UTILS_CLOCK_PLUGIN_LIBRARY=$<TARGET_FILE:MujocoRosUtilsClockPublisherPlugin>")
    endif()
    if(TARGET MujocoRosUtilsSensorPublisherPlugin)
      list(
        APPEND plugin_test_environment
        "MUJOCO_ROS_UTILS_SENSOR_PLUGIN_LIBRARY=$<TARGET_FILE:MujocoRosUtilsSensorPublisherPlugin>")
    endif()
    if(TARGET MujocoRosUtilsPosePublisherPlugin)
      list(
        APPEND plugin_test_environment
        "MUJOCO_ROS_UTILS_POSE_PLUGIN_LIBRARY=$<TARGET_FILE:MujocoRosUtilsPosePublisherPlugin>")
    endif()
    if(TARGET MujocoRosUtilsActuatorCommandPlugin)
      list(
        APPEND plugin_test_environment
        "MUJOCO_ROS_UTILS_ACTUATOR_PLUGIN_LIBRARY=$<TARGET_FILE:MujocoRosUtilsActuatorCommandPlugin>")
    endif()
    if(TARGET MujocoRosUtilsLidarPublisherPlugin)
      list(
        APPEND plugin_test_environment
        "MUJOCO_ROS_UTILS_LIDAR_PLUGIN_LIBRARY=$<TARGET_FILE:MujocoRosUtilsLidarPublisherPlugin>")
    endif()
    set_tests_properties(
      test_mujoco_plugin_loading
      PROPERTIES ENVIRONMENT "${plugin_test_environment}")
  endif()

  ament_add_gtest(
    test_mujoco_system_roadmap
    test_mujoco_system_roadmap.cpp)

  ament_add_gtest(
    test_ros_context_manager
    test_ros_context_manager.cpp)
  if(TARGET test_ros_context_manager)
    target_include_directories(
      test_ros_context_manager
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../plugin)
    ament_target_dependencies(test_ros_context_manager rclcpp)
    target_link_libraries(test_ros_context_manager MujocoRosUtilsCore)
  endif()
  if(TARGET test_mujoco_system_roadmap)
    target_include_directories(
      test_mujoco_system_roadmap
      PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    ament_target_dependencies(
      test_mujoco_system_roadmap
      hardware_interface
      pluginlib
      rclcpp)
    target_link_libraries(
      test_mujoco_system_roadmap
      ${LIB_MUJOCO})
    target_compile_definitions(
      test_mujoco_system_roadmap
      PRIVATE
        MUJOCO_ROS_UTILS_TEST_XML_DIR="${PROJECT_SOURCE_DIR}/xml")
    set_tests_properties(
      test_mujoco_system_roadmap
      PROPERTIES ENVIRONMENT
                 "MUJOCO_ROS_UTILS_TEST_XML_DIR=${PROJECT_SOURCE_DIR}/xml")
  endif()

endif()

if(MUJOCO_ROS_UTILS_BUILD_BENCHMARKS)
  add_executable(benchmark_mujoco_system benchmark_mujoco_system.cpp)
  target_include_directories(
    benchmark_mujoco_system
    PRIVATE
      ${MUJOCO_INCLUDE_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}/../include)
  ament_target_dependencies(
    benchmark_mujoco_system
    hardware_interface
    pluginlib
    rclcpp)
  target_link_libraries(
    benchmark_mujoco_system
    ${LIB_MUJOCO})
  target_compile_definitions(
    benchmark_mujoco_system
    PRIVATE
      MUJOCO_ROS_UTILS_TEST_XML_DIR="${PROJECT_SOURCE_DIR}/xml")
endif()
