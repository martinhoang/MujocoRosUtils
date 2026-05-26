#include "between_step_hook.h"

#include <functional>
#include <mutex>

namespace mujoco_ros_utils {

namespace {
std::mutex g_hook_mutex;
std::function<void(mjModel*, mjData*)> g_pending_hook;
}  // namespace

void RegisterBetweenStepHook(std::function<void(mjModel*, mjData*)> hook) {
  std::lock_guard<std::mutex> lk(g_hook_mutex);
  g_pending_hook = std::move(hook);
}

bool CheckBetweenStepHook(mjModel* m, mjData* d) {
  std::function<void(mjModel*, mjData*)> hook;
  {
    std::lock_guard<std::mutex> lk(g_hook_mutex);
    if (!g_pending_hook) return false;
    hook = std::move(g_pending_hook);
    g_pending_hook = nullptr;
  }
  hook(m, d);
  return true;
}

}  // namespace mujoco_ros_utils
