#include "between_step_hook.h"

#include <cstdio>
#include <functional>
#include <mutex>

namespace mujoco_ros_utils {

namespace {
std::mutex g_hook_mutex;
std::function<void(mjModel*, mjData*)> g_pending_hook;
void* g_core_instance = nullptr;  // set once to identify this .so instance
}  // namespace

void RegisterBetweenStepHook(std::function<void(mjModel*, mjData*)> hook) {
  if (!g_core_instance) {
    g_core_instance = (void*)&g_hook_mutex;
    fprintf(stderr, "[between_step_hook] MujocoRosUtilsCore instance: %p\n", g_core_instance);
  }
  fprintf(stderr, "[between_step_hook] Hook REGISTERED (core=%p)\n", g_core_instance);
  std::lock_guard<std::mutex> lk(g_hook_mutex);
  g_pending_hook = std::move(hook);
}

bool CheckBetweenStepHook(mjModel* m, mjData* d) {
  if (!g_core_instance) {
    g_core_instance = (void*)&g_hook_mutex;
    fprintf(stderr, "[between_step_hook] MujocoRosUtilsCore instance: %p (from Check)\n", g_core_instance);
  }
  std::function<void(mjModel*, mjData*)> hook;
  {
    std::lock_guard<std::mutex> lk(g_hook_mutex);
    if (!g_pending_hook) return false;
    hook = std::move(g_pending_hook);
    g_pending_hook = nullptr;
  }
  fprintf(stderr, "[between_step_hook] Hook EXECUTING (core=%p)\n", g_core_instance);
  hook(m, d);
  return true;
}

}  // namespace mujoco_ros_utils
