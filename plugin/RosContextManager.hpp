#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <mutex>

namespace MujocoRosUtils
{

class RosContextManager
{
public:
  static RosContextManager & instance();

  std::size_t acquire(int argc = 0, char ** argv = nullptr);
  std::size_t acquire(
      const rclcpp::InitOptions & init_options, int argc = 0, char ** argv = nullptr);
  std::size_t release() noexcept;

  std::size_t ref_count() const;
  bool is_ok() const;

  RosContextManager(const RosContextManager &) = delete;
  RosContextManager & operator=(const RosContextManager &) = delete;

private:
  RosContextManager() = default;

  mutable std::mutex mutex_;
  std::size_t ref_count_{0};
  bool owns_context_{false};
};

class RosContextLease
{
public:
  RosContextLease() = default;
  ~RosContextLease();

  RosContextLease(const RosContextLease &) = delete;
  RosContextLease & operator=(const RosContextLease &) = delete;
  RosContextLease(RosContextLease && other) noexcept;
  RosContextLease & operator=(RosContextLease && other) noexcept;

  void acquire(int argc = 0, char ** argv = nullptr);
  void acquire(
      const rclcpp::InitOptions & init_options, int argc = 0, char ** argv = nullptr);
  void release() noexcept;
  bool owns_lease() const noexcept { return owns_lease_; }

private:
  bool owns_lease_{false};
};

} // namespace MujocoRosUtils
