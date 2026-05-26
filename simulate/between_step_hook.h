#pragma once
// Between-step hook for safe model recompilation from MuJoCo plugins.
//
// MuJoCo's plugin.compute callback runs INSIDE mj_step (during mj_passive →
// mj_fwdVelocity).  Calling mj_recompile from there corrupts the in-progress
// step because mj_recompile resets d->arena, invalidating scratch allocations
// made earlier in the same step.
//
// This mechanism lets a plugin register a lambda that the simulate main loop
// runs BETWEEN steps (while sim.mtx is held, so no concurrent mj_step runs).
// The plugin's compute() prepares the mjSpec (mj_copyBack + mjs_attach/delete),
// registers the hook, and returns immediately.  The hook fires on the very next
// PhysicsLoop iteration — typically within one simulation timestep.
//
// Usage (plugin side):
//   mujoco_ros_utils::RegisterBetweenStepHook(
//       [spec, op_ptr, shared](mjModel* m, mjData* d) { ... mj_recompile(...); });
//
// Usage (simulate side):
//   if (mujoco_ros_utils::CheckBetweenStepHook(m, d)) { sim.speed_changed = true; }

#include <functional>
#include <mujoco/mujoco.h>

namespace mujoco_ros_utils {

// Register a callback to run between physics steps.  Only one may be pending
// at a time; registering a new one while one is already pending replaces the
// old one (the old one is dropped without running).
void RegisterBetweenStepHook(std::function<void(mjModel*, mjData*)> hook);

// Called by the simulate main loop each iteration (while sim.mtx is held).
// Runs the pending hook if any and returns true; returns false if there was none.
bool CheckBetweenStepHook(mjModel* m, mjData* d);

}  // namespace mujoco_ros_utils
