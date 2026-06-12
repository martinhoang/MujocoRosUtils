// PluginAggregator.cpp — thin .so for deployment into MuJoCo's mujoco_plugin/
//
// When MuJoCo loads this .so, the dynamic linker pulls in every modular
// MujocoRosUtils plugin .so listed as a link dependency.  Each modular .so
// carries its own mjPLUGIN_LIB_INIT constructor that self-registers the
// plugin, so this file needs no registration code — empty TU is sufficient.
//
// This is intentionally minimal: editing a single plugin (e.g. SceneManager)
// only recompiles that plugin's .so; this aggregator changes only when the
// set of enabled plugins changes (CMake re-link, no source recompilation).

extern "C" void _mujoco_ros_utils_aggregator_noop() {}

