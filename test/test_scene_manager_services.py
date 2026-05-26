#!/usr/bin/env python3
"""
Integration test for SceneManager ROS 2 services.

Usage (self-contained — starts and stops simulate automatically):
    source install/setup.bash
    python3 src/mujoco_ros_utils/test/test_scene_manager_services.py

  Or use the provided launch file:
    ros2 launch mujoco_ros_utils test_scene_manager.launch.py

Exit code: 0 = all tests passed, 1 = one or more tests failed.
"""

import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from mujoco_ros_utils.srv import (
    GetBodyPose,
    GetGeomProperties,
    GetModelInfo,
    SetBodyPose,
    SetGeomProperties,
    SpawnEntity,
    DespawnEntity,
    ListEntities,
)

# ── ANSI colours ─────────────────────────────────────────────────────────────
GREEN = "\033[92m"
RED   = "\033[91m"
YELLOW= "\033[93m"
RESET = "\033[0m"
BOLD  = "\033[1m"

NAMESPACE = "scene_manager"   # node_name in the XML config

# ── tolerance for floating-point comparisons ─────────────────────────────────
EPS = 1e-3


# ─────────────────────────────────────────────────────────────────────────────
class SceneManagerTester(Node):
    def __init__(self):
        super().__init__("scene_manager_tester")
        timeout = 30.0
        self._pass = 0
        self._fail = 0

        def make_client(srv_type, name):
            cli = self.create_client(srv_type, f"/{NAMESPACE}/{name}")
            self.get_logger().info(f"Waiting for /{NAMESPACE}/{name} ...")
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Service /{NAMESPACE}/{name} not available after {timeout}s")
            return cli

        self._get_body_pose     = make_client(GetBodyPose,         "get_body_pose")
        self._get_geom_props    = make_client(GetGeomProperties,   "get_geom_properties")
        self._get_model_info    = make_client(GetModelInfo,        "get_model_info")
        self._set_body_pose     = make_client(SetBodyPose,         "set_body_pose")
        self._set_geom_props    = make_client(SetGeomProperties,   "set_geom_properties")
        self._spawn             = make_client(SpawnEntity,         "spawn_entity")
        self._despawn           = make_client(DespawnEntity,       "despawn_entity")
        self._list              = make_client(ListEntities,        "list_entities")

        self.get_logger().info("All services ready — starting tests.\n")

    # ── helper ────────────────────────────────────────────────────────────────

    def _call(self, client, request, timeout_sec: float = 10.0):
        """Call a service, retrying if the future times out (e.g. after mj_recompile)."""
        deadline = time.time() + timeout_sec
        while True:
            future = client.call_async(request)
            remaining = max(0.1, deadline - time.time())
            rclpy.spin_until_future_complete(self, future, timeout_sec=min(2.0, remaining))
            if future.result() is not None:
                return future.result()
            if time.time() >= deadline:
                raise TimeoutError(f"Service call to {client.srv_name} timed out after {timeout_sec}s")
            # Service may be briefly unavailable while mj_recompile reinitialises plugins.
            time.sleep(0.2)

    def _check(self, name: str, condition: bool, detail: str = ""):
        if condition:
            self._pass += 1
            print(f"  {GREEN}✓{RESET} {name}")
        else:
            self._fail += 1
            print(f"  {RED}✗{RESET} {BOLD}{name}{RESET}  ← {detail}")

    def _approx(self, a: float, b: float, eps: float = EPS) -> bool:
        return abs(a - b) < eps

    # ── test cases ────────────────────────────────────────────────────────────

    def test_get_model_info_all(self):
        print(f"\n{BOLD}[get_model_info] no filter{RESET}")
        req = GetModelInfo.Request()
        req.filter_prefix = ""
        res = self._call(self._get_model_info, req)

        self._check("success=True",       res.success, res.message)
        self._check("nbody > 0",          res.nbody > 0)
        self._check("ngeom > 0",          res.ngeom > 0)
        self._check("static_body listed", "static_body" in res.body_names,
                    f"bodies={res.body_names}")
        self._check("dynamic_body listed","dynamic_body" in res.body_names)
        self._check("static_box_geom listed", "static_box_geom" in res.geom_names,
                    f"geoms={res.geom_names}")
        self._check("dynamic_sphere_geom listed", "dynamic_sphere_geom" in res.geom_names)

    def test_get_model_info_filter(self):
        print(f"\n{BOLD}[get_model_info] prefix='static'{RESET}")
        req = GetModelInfo.Request()
        req.filter_prefix = "static"
        res = self._call(self._get_model_info, req)

        self._check("success=True",        res.success, res.message)
        self._check("static_body present", "static_body" in res.body_names)
        self._check("dynamic_body absent", "dynamic_body" not in res.body_names,
                    f"got {res.body_names}")
        self._check("static_box_geom",     "static_box_geom" in res.geom_names)

    def test_get_body_pose_static(self):
        print(f"\n{BOLD}[get_body_pose] static_body{RESET}")
        req = GetBodyPose.Request()
        req.body_name = "static_body"
        res = self._call(self._get_body_pose, req)

        self._check("success=True",     res.success, res.message)
        self._check("is_dynamic=False", not res.is_dynamic)
        # World-frame pos from d->xpos should match MJCF position (0.3 0.0 0.5)
        self._check("pos_x ≈ 0.3", self._approx(res.pos_x, 0.3),
                    f"got {res.pos_x:.4f}")
        self._check("pos_y ≈ 0.0", self._approx(res.pos_y, 0.0),
                    f"got {res.pos_y:.4f}")
        self._check("pos_z ≈ 0.5", self._approx(res.pos_z, 0.5),
                    f"got {res.pos_z:.4f}")
        # Quaternion should be unit identity (w=1,x=y=z=0)
        self._check("rot_qw ≈ 1.0", self._approx(res.rot_qw, 1.0),
                    f"got {res.rot_qw:.4f}")
        self._check("parent_body non-empty", len(res.parent_body) > 0,
                    f"got '{res.parent_body}'")

    def test_get_body_pose_dynamic(self):
        print(f"\n{BOLD}[get_body_pose] dynamic_body{RESET}")
        req = GetBodyPose.Request()
        req.body_name = "dynamic_body"
        res = self._call(self._get_body_pose, req)

        self._check("success=True",    res.success, res.message)
        self._check("is_dynamic=True", res.is_dynamic)
        # Initial position from MJCF (0.0 0.5 0.5) — may have settled slightly
        self._check("pos_x ≈ 0.0", self._approx(res.pos_x, 0.0, eps=0.05),
                    f"got {res.pos_x:.4f}")

    def test_get_body_pose_missing(self):
        print(f"\n{BOLD}[get_body_pose] missing body{RESET}")
        req = GetBodyPose.Request()
        req.body_name = "this_body_does_not_exist"
        res = self._call(self._get_body_pose, req)
        self._check("success=False for unknown body", not res.success,
                    "expected failure for nonexistent body")

    def test_get_geom_props_box(self):
        print(f"\n{BOLD}[get_geom_properties] static_box_geom{RESET}")
        req = GetGeomProperties.Request()
        req.geom_name = "static_box_geom"
        res = self._call(self._get_geom_props, req)

        self._check("success=True",       res.success, res.message)
        self._check("geom_type = box",    res.geom_type == "box",
                    f"got '{res.geom_type}'")
        self._check("size_x ≈ 0.1",       self._approx(res.size_x, 0.1),
                    f"got {res.size_x:.4f}")
        # RGBA: (1 0 0 1) = red
        self._check("rgba_r ≈ 1.0",       self._approx(res.rgba_r, 1.0),
                    f"got {res.rgba_r:.4f}")
        self._check("rgba_g ≈ 0.0",       self._approx(res.rgba_g, 0.0),
                    f"got {res.rgba_g:.4f}")
        self._check("parent_body=static_body",
                    res.parent_body == "static_body",
                    f"got '{res.parent_body}'")

    def test_get_geom_props_sphere(self):
        print(f"\n{BOLD}[get_geom_properties] dynamic_sphere_geom{RESET}")
        req = GetGeomProperties.Request()
        req.geom_name = "dynamic_sphere_geom"
        res = self._call(self._get_geom_props, req)

        self._check("success=True",          res.success, res.message)
        self._check("geom_type = sphere",    res.geom_type == "sphere",
                    f"got '{res.geom_type}'")
        self._check("size_x ≈ 0.08",         self._approx(res.size_x, 0.08),
                    f"got {res.size_x:.4f}")
        self._check("rgba_g ≈ 1.0",          self._approx(res.rgba_g, 1.0),
                    f"got {res.rgba_g:.4f}")

    def test_get_geom_props_missing(self):
        print(f"\n{BOLD}[get_geom_properties] missing geom{RESET}")
        req = GetGeomProperties.Request()
        req.geom_name = "nonexistent_geom_xyz"
        res = self._call(self._get_geom_props, req)
        self._check("success=False for unknown geom", not res.success)

    def test_set_body_pose_static(self):
        print(f"\n{BOLD}[set_body_pose] move static_body → (0.5, 0.2, 0.7){RESET}")
        NaN = float("nan")
        req = SetBodyPose.Request()
        req.body_name = "static_body"
        req.pos_x = 0.5;  req.pos_y = 0.2;  req.pos_z = 0.7
        req.rot_qw = 0.0  # all-zero = keep orientation
        req.relative = False
        res = self._call(self._set_body_pose, req)
        self._check("set success=True", res.success, res.message)

        # Verify with get
        time.sleep(0.05)  # let sim step update d->xpos
        get_req = GetBodyPose.Request()
        get_req.body_name = "static_body"
        get_res = self._call(self._get_body_pose, get_req)
        self._check("pos_x → 0.5",  self._approx(get_res.pos_x, 0.5), f"got {get_res.pos_x:.4f}")
        self._check("pos_y → 0.2",  self._approx(get_res.pos_y, 0.2), f"got {get_res.pos_y:.4f}")
        self._check("pos_z → 0.7",  self._approx(get_res.pos_z, 0.7), f"got {get_res.pos_z:.4f}")

    def test_set_body_pose_dynamic(self):
        print(f"\n{BOLD}[set_body_pose] teleport dynamic_body → (-0.3, 0.0, 1.0){RESET}")
        req = SetBodyPose.Request()
        req.body_name = "dynamic_body"
        req.pos_x = -0.3;  req.pos_y = 0.0;  req.pos_z = 1.0
        req.rot_qw = 1.0;  req.rot_qx = 0.0;  req.rot_qy = 0.0;  req.rot_qz = 0.0
        req.relative = False
        res = self._call(self._set_body_pose, req)
        self._check("set success=True", res.success, res.message)

        time.sleep(0.05)
        get_req = GetBodyPose.Request()
        get_req.body_name = "dynamic_body"
        get_res = self._call(self._get_body_pose, get_req)
        self._check("pos_x → -0.3", self._approx(get_res.pos_x, -0.3, eps=0.02),
                    f"got {get_res.pos_x:.4f}")
        self._check("pos_z → 1.0",  self._approx(get_res.pos_z, 1.0,  eps=0.02),
                    f"got {get_res.pos_z:.4f}")

    def test_set_body_pose_relative(self):
        print(f"\n{BOLD}[set_body_pose] relative delta on ref_body{RESET}")
        # Get current pos
        get_req = GetBodyPose.Request()
        get_req.body_name = "ref_body"
        before = self._call(self._get_body_pose, get_req)
        self._check("before success=True", before.success, before.message)

        req = SetBodyPose.Request()
        req.body_name = "ref_body"
        req.pos_x = 0.1;  req.pos_y = 0.0;  req.pos_z = 0.0
        req.rot_qw = 0.0  # keep orientation
        req.relative = True
        res = self._call(self._set_body_pose, req)
        self._check("set success=True", res.success, res.message)

        time.sleep(0.05)
        after = self._call(self._get_body_pose, get_req)
        expected_x = before.pos_x + 0.1
        self._check("pos_x shifted +0.1",
                    self._approx(after.pos_x, expected_x),
                    f"expected {expected_x:.3f}, got {after.pos_x:.4f}")

    def test_set_geom_properties(self):
        print(f"\n{BOLD}[set_geom_properties] resize + recolor static_box_geom{RESET}")
        NaN = float("nan")
        req = SetGeomProperties.Request()
        req.geom_name = "static_box_geom"
        req.size_x = 0.2;  req.size_y = 0.05;  req.size_z = NaN  # keep z
        req.rgba_r = 0.0;  req.rgba_g = 0.0;   req.rgba_b = 1.0;  req.rgba_a = 0.8
        res = self._call(self._set_geom_props, req)
        self._check("set success=True", res.success, res.message)

        time.sleep(0.05)
        get_req = GetGeomProperties.Request()
        get_req.geom_name = "static_box_geom"
        get_res = self._call(self._get_geom_props, get_req)
        self._check("size_x → 0.2",  self._approx(get_res.size_x, 0.2),  f"got {get_res.size_x:.4f}")
        self._check("size_y → 0.05", self._approx(get_res.size_y, 0.05), f"got {get_res.size_y:.4f}")
        self._check("size_z unchanged (≈0.1)", self._approx(get_res.size_z, 0.1),
                    f"got {get_res.size_z:.4f}")
        self._check("rgba_b → 1.0",  self._approx(get_res.rgba_b, 1.0),  f"got {get_res.rgba_b:.4f}")
        self._check("rgba_a → 0.8",  self._approx(get_res.rgba_a, 0.8, eps=0.02),
                    f"got {get_res.rgba_a:.4f}")

    def test_spawn_and_despawn(self):
        print(f"\n{BOLD}[spawn_entity] spawn a red box{RESET}")
        box_mjcf = """
<mujoco>
  <worldbody>
    <body name="spawned_test_box" pos="0 0 0">
      <freejoint/>
      <geom name="spawned_box_geom" type="box" size="0.08 0.08 0.08"
            mass="0.3" rgba="1.0 0.5 0.0 1.0"/>
    </body>
  </worldbody>
</mujoco>
"""
        spawn_req = SpawnEntity.Request()
        spawn_req.name        = "test_box"
        spawn_req.xml_content = box_mjcf
        spawn_req.attach_to   = "world"
        spawn_req.pos_x = 0.0;  spawn_req.pos_y = -0.5;  spawn_req.pos_z = 0.5
        spawn_req.rot_qw = 1.0
        spawn_req.with_freejoint = False  # already has one in MJCF

        spawn_res = self._call(self._spawn, spawn_req)
        self._check("spawn success=True", spawn_res.success, spawn_res.message)
        self._check("spawned_body_name non-empty",
                    len(spawn_res.spawned_body_name) > 0 if spawn_res.success else True,
                    spawn_res.message)

        if spawn_res.success:
            time.sleep(0.1)

            # list_entities should now include "test_box"
            print(f"\n{BOLD}[list_entities] after spawn{RESET}")
            list_res = self._call(self._list, ListEntities.Request())
            self._check("test_box in list", "test_box" in list_res.names,
                        f"got names={list_res.names}")

            # model_info should show the new body
            info_req = GetModelInfo.Request(); info_req.filter_prefix = "test_box"
            info_res = self._call(self._get_model_info, info_req)
            self._check("spawned body visible in model_info",
                        len(info_res.body_names) > 0 or
                        any("test_box" in n for n in info_res.body_names),
                        f"bodies={info_res.body_names}")

            # despawn
            print(f"\n{BOLD}[despawn_entity] remove test_box{RESET}")
            desp_req = DespawnEntity.Request()
            desp_req.name = "test_box"
            desp_res = self._call(self._despawn, desp_req)
            self._check("despawn success=True", desp_res.success, desp_res.message)

            time.sleep(0.1)

            # list_entities should no longer include it
            list_res2 = self._call(self._list, ListEntities.Request())
            self._check("test_box removed from list", "test_box" not in list_res2.names,
                        f"got names={list_res2.names}")
        else:
            self.get_logger().warning("Skipping despawn/list checks because spawn failed.")

    def test_spawn_with_empty_name(self):
        print(f"\n{BOLD}[spawn_entity] empty name guard{RESET}")
        req = SpawnEntity.Request()
        req.name = ""
        req.xml_content = "<mujoco><worldbody><body name='x'><geom type='sphere' size='0.01'/></body></worldbody></mujoco>"
        res = self._call(self._spawn, req)
        self._check("rejected empty name", not res.success,
                    "expected failure for empty name")

    def test_despawn_unknown(self):
        print(f"\n{BOLD}[despawn_entity] unknown entity{RESET}")
        req = DespawnEntity.Request()
        req.name = "entity_that_does_not_exist_xyz"
        res = self._call(self._despawn, req)
        self._check("success=False for unknown entity", not res.success)

    # ── runner ────────────────────────────────────────────────────────────────

    def run_all(self) -> int:
        tests = [
            self.test_get_model_info_all,
            self.test_get_model_info_filter,
            self.test_get_body_pose_static,
            self.test_get_body_pose_dynamic,
            self.test_get_body_pose_missing,
            self.test_get_geom_props_box,
            self.test_get_geom_props_sphere,
            self.test_get_geom_props_missing,
            self.test_set_body_pose_static,
            self.test_set_body_pose_dynamic,
            self.test_set_body_pose_relative,
            self.test_set_geom_properties,
            self.test_spawn_and_despawn,
            self.test_spawn_with_empty_name,
            self.test_despawn_unknown,
        ]
        for t in tests:
            try:
                t()
            except Exception as e:
                print(f"  {RED}EXCEPTION in {t.__name__}: {e}{RESET}")
                self._fail += 1

        total = self._pass + self._fail
        print(f"\n{'='*50}")
        if self._fail == 0:
            print(f"{GREEN}{BOLD}ALL {self._pass}/{total} TESTS PASSED{RESET}")
        else:
            print(f"{RED}{BOLD}{self._fail}/{total} TESTS FAILED  "
                  f"({self._pass} passed){RESET}")
        print("="*50)
        return 0 if self._fail == 0 else 1


# ─────────────────────────────────────────────────────────────────────────────
def _find_simulate() -> str:
    """Return path to the simulate binary."""
    # Prefer the one in PATH
    import shutil
    sim = shutil.which("simulate")
    if sim:
        return sim
    # Fall back to MUJOCO_PATH env
    mj = os.environ.get("MUJOCO_PATH", "")
    candidate = os.path.join(mj, "bin", "simulate")
    if os.path.isfile(candidate):
        return candidate
    raise RuntimeError("Cannot find 'simulate' binary. Add it to PATH or set MUJOCO_PATH.")


def _find_world_xml() -> str:
    """Return path to the test world XML."""
    # The XML is installed to <prefix>/share/xml/, not under the ament package share.
    # Locate the install prefix via ament_index, then step up one level.
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("mujoco_ros_utils")
        # pkg_share is <prefix>/share/mujoco_ros_utils — xml/ is inside it
        candidate = os.path.join(pkg_share, "xml", "test_scene_manager_world.xml")
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    # Fallback: relative to this script (source tree)
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, "xml", "test_scene_manager_world.xml")
    if os.path.isfile(candidate):
        return os.path.realpath(candidate)
    raise RuntimeError("Cannot find test_scene_manager_world.xml")


def main():
    sim_proc = None
    exit_code = 1
    try:
        # Initialise ROS first so we can probe for an already-running simulate.
        rclpy.init()
        from mujoco_ros_utils.srv import GetBodyPose as _GBP

        # If the launch file already started simulate, services will be available
        # within the probe window.  Skip spawning our own instance to avoid two
        # conflicting simulate processes (which causes segfaults on spawn_entity).
        _probe = Node("_sm_probe")
        _probe_cli = _probe.create_client(_GBP, "/scene_manager/get_body_pose")
        services_already_up = _probe_cli.wait_for_service(timeout_sec=3.0)
        _probe.destroy_node()

        if not services_already_up:
            sim_bin  = _find_simulate()
            world    = _find_world_xml()
            hook_so  = os.path.expanduser(
                "~/.mujoco/mujoco-3.8.1/bin/mujoco_plugin/libmujoco_step_hook.so"
            )

            env = os.environ.copy()
            env.setdefault("MUJOCO_GL", "osmesa")
            if os.path.isfile(hook_so):
                existing = env.get("LD_PRELOAD", "")
                env["LD_PRELOAD"] = f"{hook_so}:{existing}" if existing else hook_so

            print(f"[test] Starting simulate: {sim_bin} {world}")
            sim_proc = subprocess.Popen(
                [sim_bin, world],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                # New process group so we can cleanly terminate it
                start_new_session=True,
            )

            # Wait for the SceneManager services to come up (up to 30 s)
            print("[test] Waiting for SceneManager services...")
            _probe2 = Node("_sm_probe2")
            deadline = time.time() + 30.0
            ready = False
            try:
                cli = _probe2.create_client(_GBP, "/scene_manager/get_body_pose")
                while time.time() < deadline:
                    if sim_proc.poll() is not None:
                        raise RuntimeError("simulate exited unexpectedly before services came up")
                    if cli.wait_for_service(timeout_sec=1.0):
                        ready = True
                        break
            finally:
                _probe2.destroy_node()

            if not ready:
                raise RuntimeError("Timed out waiting for /scene_manager/get_body_pose")

        tester = SceneManagerTester()
        exit_code = tester.run_all()

    except KeyboardInterrupt:
        exit_code = 1
    except Exception as e:
        print(f"\033[91mFatal: {e}\033[0m", file=sys.stderr)
        exit_code = 1
    finally:
        if sim_proc is not None and sim_proc.poll() is None:
            print("[test] Shutting down simulate...")
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
            try:
                sim_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
            print("[test] simulate stopped.")
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
