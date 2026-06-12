cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME mujoco_ros_utils)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/MujocoRosUtils)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT FALSE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
option(
  MUJOCO_ROS_UTILS_BUILD_TESTING
  "Build MujocoRosUtils unit and integration tests"
  ON)
option(
  MUJOCO_ROS_UTILS_BUILD_BENCHMARKS
  "Build manually-invoked MujocoRosUtils benchmark executables"
  OFF)
set(
  MUJOCO_ROS_UTILS_ENABLE_CUDA
  "AUTO"
  CACHE STRING "ImagePublisher CUDA support: AUTO, ON, or OFF")
set_property(
  CACHE MUJOCO_ROS_UTILS_ENABLE_CUDA PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${MUJOCO_ROS_UTILS_ENABLE_CUDA}" MUJOCO_ROS_UTILS_ENABLE_CUDA)
if(NOT MUJOCO_ROS_UTILS_ENABLE_CUDA MATCHES "^(AUTO|ON|OFF)$")
  message(
    FATAL_ERROR
      "MUJOCO_ROS_UTILS_ENABLE_CUDA must be AUTO, ON, or OFF, got "
      "'${MUJOCO_ROS_UTILS_ENABLE_CUDA}'")
endif()
set(
  MUJOCO_ROS_UTILS_PLUGIN_LAYOUT
  "MODULAR"
  CACHE STRING
        "MuJoCo plugin library layout: MODULAR (one DSO per plugin) or MONOLITHIC")
set_property(
  CACHE MUJOCO_ROS_UTILS_PLUGIN_LAYOUT PROPERTY STRINGS MODULAR MONOLITHIC)
string(TOUPPER
       "${MUJOCO_ROS_UTILS_PLUGIN_LAYOUT}"
       MUJOCO_ROS_UTILS_PLUGIN_LAYOUT)
if(NOT MUJOCO_ROS_UTILS_PLUGIN_LAYOUT STREQUAL "MODULAR"
   AND NOT MUJOCO_ROS_UTILS_PLUGIN_LAYOUT STREQUAL "MONOLITHIC")
  message(
    FATAL_ERROR
      "MUJOCO_ROS_UTILS_PLUGIN_LAYOUT must be MODULAR or MONOLITHIC, got "
      "'${MUJOCO_ROS_UTILS_PLUGIN_LAYOUT}'")
endif()

option(MUJOCO_ROS_UTILS_BUILD_CLOCK_PUBLISHER "Build ClockPublisher" ON)
option(MUJOCO_ROS_UTILS_BUILD_POSE_PUBLISHER "Build PosePublisher" ON)
option(MUJOCO_ROS_UTILS_BUILD_IMAGE_PUBLISHER "Build ImagePublisher" ON)
option(MUJOCO_ROS_UTILS_BUILD_ACTUATOR_COMMAND "Build ActuatorCommand" ON)
option(MUJOCO_ROS_UTILS_BUILD_EXTERNAL_FORCE "Build ExternalForce" ON)
option(MUJOCO_ROS_UTILS_BUILD_SENSOR_PUBLISHER "Build SensorPublisher" ON)
option(MUJOCO_ROS_UTILS_BUILD_LIDAR_PUBLISHER "Build LidarPublisher" ON)
option(MUJOCO_ROS_UTILS_BUILD_MIMIC_JOINT "Build MimicJoint" ON)
option(MUJOCO_ROS_UTILS_BUILD_POSE_RANDOMIZER "Build PoseRandomizer" ON)
option(MUJOCO_ROS_UTILS_BUILD_ROS2_CONTROL "Build Ros2Control" ON)
option(MUJOCO_ROS_UTILS_BUILD_SIM_DATA_AGGREGATOR
       "Build SimDataAggregator and SimRecorder" ON)
option(MUJOCO_ROS_UTILS_BUILD_SIM_PLOTTER "Build SimPlotter" ON)
option(MUJOCO_ROS_UTILS_BUILD_SCENE_MANAGER "Build SceneManager" ON)

set(
  MUJOCO_ROS_UTILS_CORE_EXTRA_SOURCES
  ""
  CACHE STRING
        "Additional source files to compile into MujocoRosUtilsCore")

# include(cmake/base.cmake)
project(${PROJECT_NAME})

function(find_mujoco)
  # MuJoCo
  if(NOT DEFINED MUJOCO_ROOT_DIR)
    set(MUJOCO_ROOT_DIR $ENV{MUJOCO_PATH})
    message(
      STATUS "Using MuJoCo path from environment variable: ${MUJOCO_ROOT_DIR}")
  endif()

  if(NOT DEFINED MUJOCO_ROOT_DIR)
    message(FATAL_ERROR "Set \"MUJOCO_ROOT_DIR\"")
  elseif(NOT EXISTS ${MUJOCO_ROOT_DIR})
    message(FATAL_ERROR "\"MUJOCO_ROOT_DIR\" is invalid path: ${MUJOCO_ROOT_DIR}")
  endif()
  set(MUJOCO_REQUIRED_VERSION 235)

  set(MUJOCO_MAIN_HEADER "${MUJOCO_ROOT_DIR}/include/mujoco/mujoco.h")
  if(NOT EXISTS "${MUJOCO_MAIN_HEADER}")
    message(
      FATAL_ERROR
        "MuJoCo main header (${MUJOCO_MAIN_HEADER}) not found, maybe MUJOCO_ROOT_DIR is badly set"
    )
  endif()
  file(READ "${MUJOCO_MAIN_HEADER}" MUJOCO_MAIN_HEADER_CONTENT)
  if(MUJOCO_MAIN_HEADER_CONTENT MATCHES ".*#define mjVERSION_HEADER ([0-9]+).*")
    set(MUJOCO_VERSION ${CMAKE_MATCH_1})
  else()
    message(
      FATAL_ERROR
        "Could not extract MuJoCo version from the header, please contact this package developers"
    )
  endif()
  if(${MUJOCO_VERSION} LESS ${MUJOCO_REQUIRED_VERSION})
    message(
      FATAL_ERROR
        "This package requires at least MuJoCo version ${MUJOCO_REQUIRED_VERSION} but you have ${MUJOCO_VERSION}"
    )
  endif()

  file(GLOB LIB_MUJOCO_GLOB "${MUJOCO_ROOT_DIR}/bin/libmujoco[0-9][0-9][0-9].so"
       "${MUJOCO_ROOT_DIR}/lib/libmujoco.so.*")
  if(EXISTS ${LIB_MUJOCO_GLOB})
    message(STATUS "MuJoCo lib found: " ${LIB_MUJOCO_GLOB})
  else()
    message(FATAL_ERROR "MuJoCo lib not found.")
  endif()
  set(LIB_MUJOCO ${LIB_MUJOCO_GLOB} PARENT_SCOPE)
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include" PARENT_SCOPE)
  set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin" PARENT_SCOPE)
endfunction()

find_mujoco()

set(AMENT_CMAKE_UNINSTALL_TARGET OFF)
find_package(ament_cmake REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PNG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ExternalForce.msg"
  "msg/ScalarStamped.msg"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  "srv/PlotCommand.srv"
  "srv/SpawnEntity.srv"
  "srv/DeleteEntity.srv"
  "srv/ListEntities.srv"
  "srv/SetBodyPose.srv"
  "srv/SetGeomProperties.srv"
  "srv/GetBodyPose.srv"
  "srv/GetGeomProperties.srv"
  "srv/GetModelInfo.srv"
  "srv/SetEqualityActive.srv"
  "srv/SetJointPositions.srv"
  "srv/ApplyBodyWrench.srv"
  "srv/GetJointState.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  builtin_interfaces)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/simulate DESTINATION bin)
install(PROGRAMS scripts/spawn_entity.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/delete_entity.py DESTINATION lib/${PROJECT_NAME})

# Do this to allow for loading xml according to the instructions in the README
# However, it is normally not done that way.
install(DIRECTORY xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

# Adding new pluginlib for MujocoSystem hardware interface
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mujoco_hardware_plugin SHARED src/mujoco_system.cpp)

target_include_directories(
  mujoco_hardware_plugin
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
  PRIVATE ${MUJOCO_INCLUDE_DIR})

ament_target_dependencies(mujoco_hardware_plugin rclcpp rclcpp_lifecycle
                          hardware_interface pluginlib control_toolbox joint_limits
                          sensor_msgs urdf)

target_link_libraries(mujoco_hardware_plugin ${LIB_MUJOCO})

install(DIRECTORY include/ DESTINATION include/mujoco_hardware_plugin)

install(
  TARGETS mujoco_hardware_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(mujoco_hardware_plugin)

#! The first argument has to be the package that contains the immediate base class of the extended plugin, and NOT the ancestral parents (like hardware_interface, etc.). Typical it is this package.
pluginlib_export_plugin_description_file(mujoco_ros_utils
                                         mujoco_hardware_plugin.xml)

ament_environment_hooks(cmake/mujoco_plugin_path.dsv)

add_subdirectory(plugin)

get_filename_component(MUJOCO_ROOT_FROM_BIN "${MUJOCO_BIN_DIR}" DIRECTORY)
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT_FROM_BIN}/simulate")
add_executable(
  mujoco_ros_simulate
  simulate/main.cc
  simulate/lodepng_compat.cpp
  "${MUJOCO_SIMULATE_DIR}/simulate.cc"
  "${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc"
  "${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc"
  "${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc")
target_include_directories(
  mujoco_ros_simulate
  PRIVATE
    "${MUJOCO_SIMULATE_DIR}"
    "${MUJOCO_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/simulate")
target_compile_definitions(
  mujoco_ros_simulate
  PRIVATE MUJOCO_PLUGIN_DIR="${MUJOCO_BIN_DIR}/mujoco_plugin")
target_link_libraries(
  mujoco_ros_simulate
  MujocoRosUtilsCore
  ${LIB_MUJOCO}
  glfw
  PNG::PNG
  ${CMAKE_DL_LIBS})
install(TARGETS mujoco_ros_simulate RUNTIME DESTINATION lib/${PROJECT_NAME})

add_subdirectory(test)

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

ament_package()
