// LD_PRELOAD library: interposes mj_step so that any registered
// between-step hooks fire BEFORE the physics step begins (not inside it).
//
// This solves the fundamental problem of calling mj_recompile from within
// plugin.compute (which runs inside mj_step):
//   - mj_recompile resets d->arena (scratch allocator), corrupting in-progress
//     scratch allocations made earlier in the same mj_step call.
//
// By intercepting mj_step at its entry point, we ensure mj_recompile runs
// BEFORE any scratch allocations for the new step have been made.
//
// Usage:
//   LD_PRELOAD=/path/to/libmujoco_step_hook.so simulate ...
//
// The library links directly against libMujocoRosUtils.so so that the hook
// registry globals are shared with the plugin (single .so instance in process).

#include <dlfcn.h>
#include <mujoco/mujoco.h>
#include "between_step_hook.h"

namespace {
// Lazily resolved pointer to the real mj_step in libmujoco.so
static void (*real_mj_step)(const mjModel*, mjData*) = nullptr;
}

// Override mj_step — runs BEFORE any scratch allocations in the step.
// Safe to call mj_recompile here because d->arena is in a clean state.
extern "C" void mj_step(const mjModel* m, mjData* d) {
  // Fire any pending between-step hook (e.g., mj_recompile for spawn/despawn).
  mujoco_ros_utils::CheckBetweenStepHook(const_cast<mjModel*>(m), d);

  // Call the real mj_step from libmujoco.so.
  if (!real_mj_step) {
    real_mj_step =
        reinterpret_cast<void (*)(const mjModel*, mjData*)>(dlsym(RTLD_NEXT, "mj_step"));
  }
  real_mj_step(m, d);
}
