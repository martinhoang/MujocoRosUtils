find_package(OpenGL REQUIRED)
find_package(OpenMP)
find_package(X11 REQUIRED)

# ── SDL2 (required by SimPlotter render thread) ───────────────────────────────
find_package(SDL2 REQUIRED)

# ── Dear ImGui + ImPlot via FetchContent ─────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
  dear_imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        3109131a882daec56a530aff540416983c240443
  GIT_SHALLOW    OFF)
FetchContent_MakeAvailable(dear_imgui)

FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG        0d4d87c3e005349c1aeb7ceacfd20a67f23c42b8
  GIT_SHALLOW    OFF)
FetchContent_MakeAvailable(implot)

# ImGui has no CMakeLists.txt — build it as a static library manually
if(NOT TARGET imgui_simplot)
  add_library(imgui_simplot STATIC
    ${dear_imgui_SOURCE_DIR}/imgui.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_draw.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_tables.cpp
    ${dear_imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp)
  target_include_directories(imgui_simplot PUBLIC
    ${dear_imgui_SOURCE_DIR}
    ${dear_imgui_SOURCE_DIR}/backends)
  # imgui_impl_opengl3 uses its own embedded glad loader (imgui_impl_opengl3_loader.h)
  # which fetches GL function pointers at runtime via SDL_GL_GetProcAddress →
  # eglGetProcAddress.  No libGL / libGLX linkage needed here.
  target_link_libraries(imgui_simplot PUBLIC SDL2::SDL2 ${CMAKE_DL_LIBS})
  set_property(TARGET imgui_simplot PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

# ImPlot also has no CMakeLists.txt
if(NOT TARGET implot_simplot)
  add_library(implot_simplot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp)
  target_include_directories(implot_simplot PUBLIC ${implot_SOURCE_DIR})
  target_link_libraries(implot_simplot PUBLIC imgui_simplot)
  set_property(TARGET implot_simplot PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

# Optional CUDA acceleration for ImagePublisher depth+cloud processing
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_library(MujocoRosUtilsCuda STATIC depth_processing_cuda.cu)
  set_target_properties(MujocoRosUtilsCuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
    CUDA_ARCHITECTURES native
    POSITION_INDEPENDENT_CODE ON)
  target_include_directories(MujocoRosUtilsCuda PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(MujocoRosUtilsCuda PUBLIC HAS_CUDA=1)
  message(STATUS "CUDA found — ImagePublisher CUDA acceleration enabled")
else()
  message(STATUS "CUDA not found — ImagePublisher CUDA acceleration disabled")
endif()

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
  LidarPublisher.cpp
  mujoco_utils.cpp
  MimicJoint.cpp
  PoseRandomizer.cpp
  RosControl.cpp
  SimDataAggregator.cpp
  SimRecorder.cpp
  SimPlotter.cpp
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
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2_ros
  sensor_msgs
  cv_bridge
  image_geometry
  trajectory_msgs
  builtin_interfaces
  hardware_interface
  controller_manager
  image_transport
  control_toolbox
  joint_limits
  urdf
  rosbag2_cpp
  rosbag2_storage
  ament_index_cpp)
add_dependencies(MujocoRosUtils ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(MujocoRosUtils PUBLIC ${cpp_typesupport_target}
                                            ${ament_LIBRARIES})
target_link_libraries(MujocoRosUtils PUBLIC ${LIB_MUJOCO} glfw)
target_link_libraries(MujocoRosUtils PUBLIC OpenGL::GL)
target_link_libraries(MujocoRosUtils PUBLIC imgui_simplot implot_simplot SDL2::SDL2 X11::X11)
if(OpenMP_CXX_FOUND)
  target_link_libraries(MujocoRosUtils PUBLIC OpenMP::OpenMP_CXX)
endif()

if(CMAKE_CUDA_COMPILER)
  target_link_libraries(MujocoRosUtils PRIVATE MujocoRosUtilsCuda CUDA::cudart)
  target_compile_definitions(MujocoRosUtils PRIVATE HAS_CUDA=1)
endif()

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
