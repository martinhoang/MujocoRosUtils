include(CMakeParseArguments)
include(GNUInstallDirs)

set(MUJOCO_ROS_UTILS_PLUGIN_INSTALL_DIR
    "lib/${PROJECT_NAME}/plugins")

function(mujoco_ros_utils_configure_library target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(
    ${target}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
    PRIVATE ${PROJECT_SOURCE_DIR}/plugin
            ${PROJECT_SOURCE_DIR}/simulate
            ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(${target} ${LIB_MUJOCO})
  set_target_properties(
    ${target}
    PROPERTIES INSTALL_RPATH "$ORIGIN"
               INSTALL_RPATH_USE_LINK_PATH TRUE)
endfunction()

function(mujoco_ros_utils_link_interfaces target)
  add_dependencies(${target} ${PROJECT_NAME})
  target_link_libraries(${target} ${MUJOCO_ROS_UTILS_TYPESUPPORT_TARGET})
endfunction()

function(mujoco_ros_utils_add_plugin key)
  set(options USE_INTERFACES)
  set(one_value_args SOURCE REGISTRATION OUTPUT_NAME)
  set(multi_value_args EXTRA_SOURCES AMENT_DEPENDENCIES LINK_LIBRARIES)
  cmake_parse_arguments(
    MRU "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT MRU_SOURCE OR NOT MRU_REGISTRATION)
    message(FATAL_ERROR "Plugin ${key} requires SOURCE and REGISTRATION")
  endif()

  set_property(GLOBAL APPEND PROPERTY MUJOCO_ROS_UTILS_PLUGIN_KEYS ${key})
  set_property(GLOBAL APPEND PROPERTY MUJOCO_ROS_UTILS_PLUGIN_SOURCES
                                      "${MRU_SOURCE}" ${MRU_EXTRA_SOURCES})
  set_property(GLOBAL APPEND PROPERTY MUJOCO_ROS_UTILS_PLUGIN_DEPENDENCIES
                                      ${MRU_AMENT_DEPENDENCIES})
  set_property(GLOBAL APPEND PROPERTY MUJOCO_ROS_UTILS_PLUGIN_LINK_LIBRARIES
                                      ${MRU_LINK_LIBRARIES})
  if(MRU_USE_INTERFACES)
    set_property(GLOBAL PROPERTY MUJOCO_ROS_UTILS_USES_INTERFACES TRUE)
  endif()

  if(NOT MUJOCO_ROS_UTILS_PLUGIN_LAYOUT STREQUAL "MODULAR")
    return()
  endif()

  set(target "MujocoRosUtils${key}Plugin")
  add_library(
    ${target} SHARED
    "${MRU_SOURCE}"
    ${MRU_EXTRA_SOURCES}
    "${MRU_REGISTRATION}")
  mujoco_ros_utils_configure_library(${target})
  target_link_libraries(
    ${target}
    MujocoRosUtilsCore
    ${MRU_LINK_LIBRARIES})
  if(MRU_AMENT_DEPENDENCIES)
    ament_target_dependencies(${target} ${MRU_AMENT_DEPENDENCIES})
  endif()
  if(MRU_USE_INTERFACES)
    mujoco_ros_utils_link_interfaces(${target})
  endif()
  if(MRU_OUTPUT_NAME)
    set_target_properties(${target} PROPERTIES OUTPUT_NAME "${MRU_OUTPUT_NAME}")
  endif()

  set_property(GLOBAL APPEND PROPERTY MUJOCO_ROS_UTILS_PLUGIN_TARGETS ${target})
endfunction()

function(mujoco_ros_utils_install_plugin_targets)
  set(targets ${ARGN})
  if(NOT targets)
    return()
  endif()

  install(
    TARGETS ${targets}
    LIBRARY DESTINATION "${MUJOCO_ROS_UTILS_PLUGIN_INSTALL_DIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
endfunction()
