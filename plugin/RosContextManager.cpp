#include "RosContextManager.hpp"

#include <stdexcept>
#include <utility>

namespace MujocoRosUtils
{

RosContextManager & RosContextManager::instance()
{
  static RosContextManager manager;
  return manager;
}

std::size_t RosContextManager::acquire(int argc, char ** argv)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(ref_count_ == 0)
  {
    owns_context_ = !rclcpp::ok();
    if(owns_context_)
    {
      rclcpp::init(argc, argv);
    }
  }
  ++ref_count_;
  return ref_count_;
}

std::size_t RosContextManager::acquire(
    const rclcpp::InitOptions & init_options, int argc, char ** argv)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(ref_count_ == 0)
  {
    owns_context_ = !rclcpp::ok();
    if(owns_context_)
    {
      rclcpp::init(argc, argv, init_options);
    }
  }
  ++ref_count_;
  return ref_count_;
}

std::size_t RosContextManager::release() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(ref_count_ == 0)
  {
    return 0;
  }

  --ref_count_;
  if(ref_count_ == 0)
  {
    if(owns_context_ && rclcpp::ok())
    {
      try
      {
        rclcpp::shutdown();
      }
      catch(...)
      {
        // Destruction paths must not terminate the process.
      }
    }
    owns_context_ = false;
  }
  return ref_count_;
}

std::size_t RosContextManager::ref_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ref_count_;
}

bool RosContextManager::is_ok() const
{
  return rclcpp::ok();
}

RosContextLease::~RosContextLease()
{
  release();
}

RosContextLease::RosContextLease(RosContextLease && other) noexcept
    : owns_lease_(std::exchange(other.owns_lease_, false))
{}

RosContextLease & RosContextLease::operator=(RosContextLease && other) noexcept
{
  if(this != &other)
  {
    release();
    owns_lease_ = std::exchange(other.owns_lease_, false);
  }
  return *this;
}

void RosContextLease::acquire(int argc, char ** argv)
{
  if(owns_lease_)
  {
    throw std::logic_error("ROS context lease already acquired");
  }
  RosContextManager::instance().acquire(argc, argv);
  owns_lease_ = true;
}

void RosContextLease::acquire(
    const rclcpp::InitOptions & init_options, int argc, char ** argv)
{
  if(owns_lease_)
  {
    throw std::logic_error("ROS context lease already acquired");
  }
  RosContextManager::instance().acquire(init_options, argc, argv);
  owns_lease_ = true;
}

void RosContextLease::release() noexcept
{
  if(owns_lease_)
  {
    RosContextManager::instance().release();
    owns_lease_ = false;
  }
}

} // namespace MujocoRosUtils
