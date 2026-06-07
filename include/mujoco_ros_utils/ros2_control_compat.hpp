#pragma once

#include <string>
#include <type_traits>
#include <utility>

#ifndef MUJOCO_ROS_UTILS_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS
  #if __has_include(<hardware_interface/types/hardware_component_interface_params.hpp>)
    #define MUJOCO_ROS_UTILS_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS 1
  #else
    #define MUJOCO_ROS_UTILS_HAS_HARDWARE_COMPONENT_INTERFACE_PARAMS 0
  #endif
#endif

namespace MujocoRosUtils
{
namespace ros2_control_compat
{

template<typename T, typename = void>
struct HasDataType : std::false_type
{};

template<typename T>
struct HasDataType<T, std::void_t<decltype(std::declval<const T &>().data_type)>> : std::true_type
{};

template<typename InterfaceInfo>
bool is_scalar_double(const InterfaceInfo & info)
{
  if constexpr(HasDataType<InterfaceInfo>::value)
  {
    return info.data_type.empty() || info.data_type == "double";
  }
  return true;
}

} // namespace ros2_control_compat
} // namespace MujocoRosUtils
