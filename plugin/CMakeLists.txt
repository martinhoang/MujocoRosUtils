add_library(
  MujocoRosUtils SHARED
  ClockPublisher.cpp
  PosePublisher.cpp
  ImagePublisher.cpp
  depth_conversions.hpp
  depth_traits.hpp
  ActuatorCommand.cpp
  ExternalForce.cpp
  SensorPublisher.cpp
  mujoco_utils.cpp
  MimicJoint.cpp
  RosControl.cpp
  # mujoco_ros2_control.cpp
  )
target_include_directories(
  MujocoRosUtils PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
target_include_directories(MujocoRosUtils PRIVATE ${MUJOCO_INCLUDE_DIR})
target_include_directories(MujocoRosUtils PUBLIC ${ament_INCLUDE_DIRS})
ament_target_dependencies(
  MujocoRosUtils
  PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  sensor_msgs
  cv_bridge
  image_geometry
  trajectory_msgs
  builtin_interfaces
  hardware_interface
  controller_manager)
add_dependencies(MujocoRosUtils ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(MujocoRosUtils PUBLIC ${cpp_typesupport_target}
                                            ${ament_LIBRARIES})
target_link_libraries(MujocoRosUtils PUBLIC ${LIB_MUJOCO} glfw)
target_link_libraries(MujocoRosUtils PUBLIC OpenMP::OpenMP_CXX)

add_library(MujocoRosUtilsPlugin SHARED Plugin.cpp)
target_include_directories(MujocoRosUtilsPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(MujocoRosUtilsPlugin PUBLIC MujocoRosUtils)
set_target_properties(
  MujocoRosUtils MujocoRosUtilsPlugin
  PROPERTIES INSTALL_RPATH "${MUJOCO_BIN_DIR}:${MUJOCO_LIB_DIR}")

set_target_properties(MujocoRosUtils MujocoRosUtilsPlugin
                      PROPERTIES INSTALL_RPATH "$ORIGIN")

install(
  TARGETS MujocoRosUtils MujocoRosUtilsPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION ${MUJOCO_BIN_DIR}/mujoco_plugin
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
