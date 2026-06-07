// Reproducible microbenchmark for
// MuJoCoSystem read/write throughput.
//
// Build:  cmake ... -DMUJOCO_ROS_UTILS_BUILD_BENCHMARKS=ON
// Run:    ./benchmark_mujoco_system [--warmup N] [--iterations N]
//
// Output: key=value lines on stdout suitable for scripting and CI.
// Exit 0 on success, non-zero on any failure.

#include "mujoco_system_interface.hpp"
#include "roadmap_test_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace
{

using hardware_interface::return_type;
using mujoco_ros2_control::MujocoSystemInterface;
using mujoco_ros_utils::test::MujocoModel;

using Clock = std::chrono::steady_clock;
using Nanos = std::chrono::nanoseconds;

struct Config
{
  int warmup{100};
  int iterations{1000};
};

Config parse_args(int argc, char **argv)
{
  Config cfg;
  for (int i = 1; i < argc; ++i)
  {
    std::string arg(argv[i]);
    if (arg == "--warmup" && i + 1 < argc)
    {
      cfg.warmup = std::stoi(argv[++i]);
    }
    else if (arg == "--iterations" && i + 1 < argc)
    {
      cfg.iterations = std::stoi(argv[++i]);
    }
    else if (arg == "--help" || arg == "-h")
    {
      std::cout << "Usage: " << argv[0]
                << " [--warmup N] [--iterations N]\n";
      std::exit(0);
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << "\n";
      std::exit(2);
    }
  }
  if (cfg.warmup < 0)
  {
    cfg.warmup = 0;
  }
  if (cfg.iterations < 1)
  {
    cfg.iterations = 1;
  }
  return cfg;
}

struct Sample
{
  long long elapsed_ns;
  double mean_ns;
  double ops_per_sec;
};

Sample measure(const std::string &label, int n,
               const std::function<return_type()> &fn)
{
  auto t0 = Clock::now();
  for (int i = 0; i < n; ++i)
  {
    if (fn() != return_type::OK)
    {
      std::cerr << "FAIL: " << label << " returned error at iteration " << i
                << "\n";
      std::exit(1);
    }
  }
  auto t1 = Clock::now();
  long long elapsed = std::chrono::duration_cast<Nanos>(t1 - t0).count();
  const auto nonzero_elapsed = std::max<long long>(elapsed, 1);
  return {elapsed, static_cast<double>(elapsed) / static_cast<double>(n),
          static_cast<double>(n) * 1e9 / static_cast<double>(nonzero_elapsed)};
}

void report(const std::string &prefix, const Sample &s)
{
  std::cout << prefix << "_elapsed_ns=" << s.elapsed_ns << "\n";
  std::cout << prefix << "_mean_ns=" << s.mean_ns << "\n";
  std::cout << prefix << "_ops_per_sec=" << s.ops_per_sec << "\n";
}

}  // namespace

int main(int argc, char **argv)
{
  Config cfg = parse_args(argc, argv);

  // --- setup ----------------------------------------------------------
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  MujocoModel model("test_ros2_control_interfaces.xml");
  auto node = std::make_shared<rclcpp::Node>("benchmark_mujoco_system");
  pluginlib::ClassLoader<MujocoSystemInterface> loader(
    "mujoco_ros_utils", "mujoco_ros2_control::MujocoSystemInterface");
  auto system = loader.createSharedInstance("mujoco_ros2_control/MujocoSystem");
  auto info =
    mujoco_ros_utils::test::hardware_info("test_ros2_control_interfaces.urdf");

  if (!system->initialize(node, model.model(), model.data(), info))
  {
    std::cerr << "FAIL: MuJoCoSystem::initialize returned false\n";
    return 1;
  }

  // Exercise the same interface-export path used by controller managers.
  system->export_state_interfaces();
  system->export_command_interfaces();

  // Activate so that both read and write operate fully.
  using rclcpp_lifecycle::State;
  system->on_activate(State(2, "inactive"));

  // --- warmup ---------------------------------------------------------
  for (int i = 0; i < cfg.warmup; ++i)
  {
    if (system->read(rclcpp::Time(0, 0),
                     rclcpp::Duration::from_seconds(0.001))
        != return_type::OK)
    {
      std::cerr << "FAIL: read during warmup at iteration " << i << "\n";
      return 1;
    }
    if (system->write(rclcpp::Time(0, 0),
                      rclcpp::Duration::from_seconds(0.001))
        != return_type::OK)
    {
      std::cerr << "FAIL: write during warmup at iteration " << i << "\n";
      return 1;
    }
  }

  // --- benchmark read -------------------------------------------------
  auto read_sample = measure("read", cfg.iterations, [&]() {
    return system->read(rclcpp::Time(1, 0),
                        rclcpp::Duration::from_seconds(0.001));
  });

  // --- benchmark write ------------------------------------------------
  auto write_sample = measure("write", cfg.iterations, [&]() {
    return system->write(rclcpp::Time(1, 0),
                         rclcpp::Duration::from_seconds(0.001));
  });

  // --- benchmark combined read+write ---------------------------------
  auto rw_sample = measure("rw", cfg.iterations, [&]() {
    if (system->read(rclcpp::Time(1, 0),
                     rclcpp::Duration::from_seconds(0.001))
        != return_type::OK)
    {
      return return_type::ERROR;
    }
    return system->write(rclcpp::Time(1, 0),
                         rclcpp::Duration::from_seconds(0.001));
  });

  // --- report ---------------------------------------------------------
  std::cout << "warmup_iterations=" << cfg.warmup << "\n";
  std::cout << "measure_iterations=" << cfg.iterations << "\n";
  report("read", read_sample);
  report("write", write_sample);
  report("rw", rw_sample);

  // --- teardown -------------------------------------------------------
  system->on_deactivate(State(3, "active"));
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  return 0;
}
