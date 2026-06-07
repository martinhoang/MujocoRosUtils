include("${PROJECT_SOURCE_DIR}/cmake/MujocoRosUtilsPlugins.cmake")

rosidl_get_typesupport_target(
  MUJOCO_ROS_UTILS_TYPESUPPORT_TARGET
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

set(core_sources
    mujoco_utils.cpp
    RosContextManager.cpp
    ../simulate/between_step_hook.cpp)
foreach(extra_source IN LISTS MUJOCO_ROS_UTILS_CORE_EXTRA_SOURCES)
  if(NOT IS_ABSOLUTE "${extra_source}")
    set(extra_source "${PROJECT_SOURCE_DIR}/${extra_source}")
  endif()
  if(NOT EXISTS "${extra_source}")
    message(
      FATAL_ERROR
        "MUJOCO_ROS_UTILS_CORE_EXTRA_SOURCES entry does not exist: ${extra_source}")
  endif()
  list(APPEND core_sources "${extra_source}")
endforeach()

add_library(MujocoRosUtilsCore SHARED ${core_sources})
mujoco_ros_utils_configure_library(MujocoRosUtilsCore)
ament_target_dependencies(MujocoRosUtilsCore rclcpp)

if(MUJOCO_ROS_UTILS_BUILD_CLOCK_PUBLISHER)
  find_package(rosgraph_msgs REQUIRED)
  mujoco_ros_utils_add_plugin(
    ClockPublisher
    SOURCE ClockPublisher.cpp
    REGISTRATION registrations/ClockPublisherRegistration.cpp
    AMENT_DEPENDENCIES rclcpp rosgraph_msgs builtin_interfaces)
endif()

if(MUJOCO_ROS_UTILS_BUILD_POSE_PUBLISHER)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  mujoco_ros_utils_add_plugin(
    PosePublisher
    SOURCE PosePublisher.cpp
    REGISTRATION registrations/PosePublisherRegistration.cpp
    AMENT_DEPENDENCIES rclcpp geometry_msgs tf2 tf2_ros)
endif()

if(MUJOCO_ROS_UTILS_BUILD_IMAGE_PUBLISHER)
  find_package(OpenGL REQUIRED)
  find_package(OpenMP REQUIRED)
  find_package(glfw3 3.3 REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_geometry REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(sensor_msgs REQUIRED)

  set(image_link_libraries OpenGL::GL glfw OpenMP::OpenMP_CXX)
  set(MUJOCO_ROS_UTILS_CUDA_ENABLED OFF)
  if(NOT MUJOCO_ROS_UTILS_ENABLE_CUDA STREQUAL "OFF")
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
      enable_language(CUDA)
      find_package(CUDAToolkit REQUIRED)
      add_library(MujocoRosUtilsCuda STATIC depth_processing_cuda.cu)
      set_target_properties(
        MujocoRosUtilsCuda
        PROPERTIES CUDA_SEPARABLE_COMPILATION OFF
                   CUDA_ARCHITECTURES native
                   POSITION_INDEPENDENT_CODE ON)
      target_include_directories(MujocoRosUtilsCuda PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
      target_compile_definitions(MujocoRosUtilsCuda PUBLIC HAS_CUDA=1)
      list(APPEND image_link_libraries MujocoRosUtilsCuda CUDA::cudart)
      set(MUJOCO_ROS_UTILS_CUDA_ENABLED ON)
      message(STATUS "CUDA found - ImagePublisher CUDA acceleration enabled")
    elseif(MUJOCO_ROS_UTILS_ENABLE_CUDA STREQUAL "ON")
      message(FATAL_ERROR "CUDA support was requested but no CUDA compiler was found")
    else()
      message(STATUS "CUDA not found - ImagePublisher CUDA acceleration disabled")
    endif()
  else()
    message(STATUS "ImagePublisher CUDA acceleration explicitly disabled")
  endif()

  mujoco_ros_utils_add_plugin(
    ImagePublisher
    SOURCE ImagePublisher.cpp
    REGISTRATION registrations/ImagePublisherRegistration.cpp
    AMENT_DEPENDENCIES
      rclcpp
      sensor_msgs
      cv_bridge
      image_geometry
      image_transport
    LINK_LIBRARIES ${image_link_libraries})
endif()

if(MUJOCO_ROS_UTILS_BUILD_ACTUATOR_COMMAND)
  find_package(sensor_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  mujoco_ros_utils_add_plugin(
    ActuatorCommand
    SOURCE ActuatorCommand.cpp
    REGISTRATION registrations/ActuatorCommandRegistration.cpp
    AMENT_DEPENDENCIES rclcpp sensor_msgs std_msgs trajectory_msgs)
endif()

if(MUJOCO_ROS_UTILS_BUILD_EXTERNAL_FORCE)
  mujoco_ros_utils_add_plugin(
    ExternalForce
    USE_INTERFACES
    SOURCE ExternalForce.cpp
    REGISTRATION registrations/ExternalForceRegistration.cpp
    AMENT_DEPENDENCIES rclcpp)
endif()

if(MUJOCO_ROS_UTILS_BUILD_SENSOR_PUBLISHER)
  mujoco_ros_utils_add_plugin(
    SensorPublisher
    USE_INTERFACES
    SOURCE SensorPublisher.cpp
    REGISTRATION registrations/SensorPublisherRegistration.cpp
    AMENT_DEPENDENCIES rclcpp geometry_msgs)
endif()

if(MUJOCO_ROS_UTILS_BUILD_LIDAR_PUBLISHER)
  find_package(OpenMP REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  mujoco_ros_utils_add_plugin(
    LidarPublisher
    SOURCE LidarPublisher.cpp
    REGISTRATION registrations/LidarPublisherRegistration.cpp
    AMENT_DEPENDENCIES rclcpp geometry_msgs sensor_msgs tf2 tf2_ros
    LINK_LIBRARIES OpenMP::OpenMP_CXX)
endif()

if(MUJOCO_ROS_UTILS_BUILD_MIMIC_JOINT)
  mujoco_ros_utils_add_plugin(
    MimicJoint
    SOURCE MimicJoint.cpp
    REGISTRATION registrations/MimicJointRegistration.cpp)
endif()

if(MUJOCO_ROS_UTILS_BUILD_POSE_RANDOMIZER)
  mujoco_ros_utils_add_plugin(
    PoseRandomizer
    SOURCE PoseRandomizer.cpp
    REGISTRATION registrations/PoseRandomizerRegistration.cpp)
endif()

if(MUJOCO_ROS_UTILS_BUILD_ROS2_CONTROL)
  find_package(ament_index_cpp REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(std_srvs REQUIRED)
  mujoco_ros_utils_add_plugin(
    Ros2Control
    SOURCE RosControl.cpp
    REGISTRATION registrations/Ros2ControlRegistration.cpp
    AMENT_DEPENDENCIES
      ament_index_cpp
      controller_manager
      control_toolbox
      hardware_interface
      joint_limits
      pluginlib
      rclcpp
      std_srvs
      urdf)
endif()

if(MUJOCO_ROS_UTILS_BUILD_SIM_DATA_AGGREGATOR)
  find_package(ament_index_cpp REQUIRED)
  find_package(HDF5 REQUIRED COMPONENTS C)
  find_package(rosbag2_cpp REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  mujoco_ros_utils_add_plugin(
    SimDataAggregator
    USE_INTERFACES
    SOURCE SimDataAggregator.cpp
    EXTRA_SOURCES SimRecorder.cpp
    REGISTRATION registrations/SimDataAggregatorRegistration.cpp
    AMENT_DEPENDENCIES
      ament_index_cpp
      rclcpp
      rosbag2_cpp
      rosbag2_storage
      sensor_msgs
      std_srvs
    LINK_LIBRARIES HDF5::HDF5)
endif()

if(MUJOCO_ROS_UTILS_BUILD_SIM_PLOTTER)
  find_package(OpenGL REQUIRED)
  find_package(SDL2 REQUIRED)
  find_package(X11 REQUIRED)

  include(FetchContent)
  FetchContent_Declare(
    dear_imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG 3109131a882daec56a530aff540416983c240443
    GIT_SHALLOW OFF)
  FetchContent_MakeAvailable(dear_imgui)

  FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG 0d4d87c3e005349c1aeb7ceacfd20a67f23c42b8
    GIT_SHALLOW OFF)
  FetchContent_MakeAvailable(implot)

  add_library(
    imgui_simplot STATIC
    ${dear_imgui_SOURCE_DIR}/imgui.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_draw.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_tables.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp)
  target_include_directories(
    imgui_simplot
    PUBLIC ${dear_imgui_SOURCE_DIR}
           ${dear_imgui_SOURCE_DIR}/backends)
  target_link_libraries(imgui_simplot SDL2::SDL2 ${CMAKE_DL_LIBS})
  set_property(TARGET imgui_simplot PROPERTY POSITION_INDEPENDENT_CODE ON)

  add_library(
    implot_simplot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp)
  target_include_directories(implot_simplot PUBLIC ${implot_SOURCE_DIR})
  target_link_libraries(implot_simplot imgui_simplot)
  set_property(TARGET implot_simplot PROPERTY POSITION_INDEPENDENT_CODE ON)

  mujoco_ros_utils_add_plugin(
    SimPlotter
    USE_INTERFACES
    SOURCE SimPlotter.cpp
    REGISTRATION registrations/SimPlotterRegistration.cpp
    AMENT_DEPENDENCIES rclcpp
    LINK_LIBRARIES
      OpenGL::GL
      SDL2::SDL2
      X11::X11
      imgui_simplot
      implot_simplot
      ${CMAKE_DL_LIBS})
endif()

if(MUJOCO_ROS_UTILS_BUILD_SCENE_MANAGER)
  mujoco_ros_utils_add_plugin(
    SceneManager
    USE_INTERFACES
    SOURCE SceneManager.cpp
    REGISTRATION registrations/SceneManagerRegistration.cpp
    AMENT_DEPENDENCIES rclcpp)
endif()

get_property(
  plugin_targets GLOBAL PROPERTY MUJOCO_ROS_UTILS_PLUGIN_TARGETS)

if(MUJOCO_ROS_UTILS_PLUGIN_LAYOUT STREQUAL "MONOLITHIC")
  get_property(
    plugin_sources GLOBAL PROPERTY MUJOCO_ROS_UTILS_PLUGIN_SOURCES)
  get_property(
    plugin_dependencies GLOBAL PROPERTY MUJOCO_ROS_UTILS_PLUGIN_DEPENDENCIES)
  get_property(
    plugin_link_libraries GLOBAL PROPERTY MUJOCO_ROS_UTILS_PLUGIN_LINK_LIBRARIES)
  get_property(
    plugin_keys GLOBAL PROPERTY MUJOCO_ROS_UTILS_PLUGIN_KEYS)
  get_property(
    uses_interfaces GLOBAL PROPERTY MUJOCO_ROS_UTILS_USES_INTERFACES)

  list(REMOVE_DUPLICATES plugin_sources)
  list(REMOVE_DUPLICATES plugin_dependencies)
  list(REMOVE_DUPLICATES plugin_link_libraries)

  add_library(MujocoRosUtils SHARED ${plugin_sources})
  mujoco_ros_utils_configure_library(MujocoRosUtils)
  target_link_libraries(
    MujocoRosUtils
    MujocoRosUtilsCore
    ${plugin_link_libraries})
  if(plugin_dependencies)
    ament_target_dependencies(MujocoRosUtils ${plugin_dependencies})
  endif()
  if(uses_interfaces)
    mujoco_ros_utils_link_interfaces(MujocoRosUtils)
  endif()
  if(MUJOCO_ROS_UTILS_CUDA_ENABLED AND MUJOCO_ROS_UTILS_BUILD_IMAGE_PUBLISHER)
    target_compile_definitions(MujocoRosUtils PRIVATE HAS_CUDA=1)
  endif()

  add_library(
    MujocoRosUtilsPlugin SHARED
    registrations/MonolithicRegistration.cpp)
  mujoco_ros_utils_configure_library(MujocoRosUtilsPlugin)
  target_link_libraries(MujocoRosUtilsPlugin MujocoRosUtils)
  if(plugin_dependencies)
    ament_target_dependencies(MujocoRosUtilsPlugin ${plugin_dependencies})
  endif()
  foreach(plugin_key IN LISTS plugin_keys)
    string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" plugin_define "${plugin_key}")
    string(TOUPPER "${plugin_define}" plugin_define)
    target_compile_definitions(
      MujocoRosUtilsPlugin
      PRIVATE "MUJOCO_ROS_UTILS_ENABLE_${plugin_define}=1")
  endforeach()
  set(plugin_targets MujocoRosUtils MujocoRosUtilsPlugin)
elseif(CMAKE_CUDA_COMPILER AND MUJOCO_ROS_UTILS_BUILD_IMAGE_PUBLISHER)
  target_compile_definitions(
    MujocoRosUtilsImagePublisherPlugin PRIVATE HAS_CUDA=1)
endif()

add_library(mujoco_step_hook SHARED ../simulate/mujoco_step_hook.cpp)
target_include_directories(
  mujoco_step_hook
  PRIVATE ${MUJOCO_INCLUDE_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/../simulate)
target_link_libraries(
  mujoco_step_hook
  MujocoRosUtilsCore
  ${LIB_MUJOCO}
  ${CMAKE_DL_LIBS})
set_target_properties(
  mujoco_step_hook
  PROPERTIES INSTALL_RPATH "$ORIGIN"
             INSTALL_RPATH_USE_LINK_PATH TRUE)

mujoco_ros_utils_install_plugin_targets(
  MujocoRosUtilsCore
  ${plugin_targets}
  mujoco_step_hook)

message(
  STATUS
    "MujocoRosUtils plugin layout: ${MUJOCO_ROS_UTILS_PLUGIN_LAYOUT}; "
    "plugin targets: ${plugin_targets}")
