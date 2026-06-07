#!/usr/bin/env python3
"""Contract tests for roadmap MJCF and ros2_control fixtures."""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XML_DIR = ROOT / "xml"
MAPPING_RE = re.compile(
    r"^mujoco_(?P<kind>sensor|actuator):(?P<name>[A-Za-z_][A-Za-z0-9_.-]*)"
    r"(?:\[(?P<index>[0-9]+)\])?$"
)

PROFILE_INTERFACES = {
    "imu": {
        "orientation.x", "orientation.y", "orientation.z", "orientation.w",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
    },
    "force_torque": {
        "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z",
    },
    "range": {"range"},
    "pose": {
        "position.x", "position.y", "position.z",
        "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    },
}


def params(component):
    return {item.attrib["name"]: (item.text or "").strip() for item in component.findall("param")}


def plugin_config(component):
    return {item.attrib["key"]: item.attrib["value"] for item in component.findall("config")}


def interface_names(component, tag):
    return {item.attrib["name"] for item in component.findall(tag)}


class RoadmapFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mjcf = ET.parse(XML_DIR / "test_ros2_control_interfaces.xml").getroot()
        cls.urdf = ET.parse(XML_DIR / "test_ros2_control_interfaces.urdf").getroot()
        cls.control = cls.urdf.find("ros2_control")
        cls.sensor_names = {
            sensor.attrib["name"] for sensor in cls.mjcf.findall("./sensor/*")
        }
        cls.sensor_dims = {
            sensor.attrib["name"]: int(sensor.attrib.get("dim", {
                "framequat": "4", "gyro": "3", "accelerometer": "3",
                "force": "3", "torque": "3", "rangefinder": "1",
                "framepos": "3",
            }.get(sensor.tag, "1")))
            for sensor in cls.mjcf.findall("./sensor/*")
        }
        cls.actuator_names = {
            actuator.attrib["name"] for actuator in cls.mjcf.findall("./actuator/*")
            if "name" in actuator.attrib
        }

    def test_all_sensor_profiles_have_exact_interfaces(self):
        sensors = {item.attrib["name"]: item for item in self.control.findall("sensor")}
        self.assertEqual(
            {params(item)["profile"] for item in sensors.values()},
            {"imu", "force_torque", "range", "pose", "generic"},
        )
        for component in sensors.values():
            profile = params(component)["profile"]
            if profile in PROFILE_INTERFACES:
                self.assertEqual(
                    interface_names(component, "state_interface"),
                    PROFILE_INTERFACES[profile],
                    component.attrib["name"],
                )

    def test_profile_sources_exist_with_expected_dimensions(self):
        expected = {
            "orientation_sensor": 4,
            "angular_velocity_sensor": 3,
            "linear_acceleration_sensor": 3,
            "force_sensor": 3,
            "torque_sensor": 3,
            "range_sensor": 1,
            "position_sensor": 3,
        }
        for component in self.control.findall("sensor"):
            for key, source in params(component).items():
                if key in expected:
                    self.assertIn(source, self.sensor_names)
                    self.assertEqual(self.sensor_dims[source], expected[key], key)

    def test_generic_sensor_mappings_are_resolvable(self):
        component = next(
            item for item in self.control.findall("sensor")
            if params(item).get("profile") == "generic"
        )
        component_params = params(component)
        for interface in interface_names(component, "state_interface"):
            match = MAPPING_RE.fullmatch(component_params[f"state.{interface}"])
            self.assertIsNotNone(match)
            self.assertEqual(match["kind"], "sensor")
            self.assertIn(match["name"], self.sensor_names)
            self.assertLess(int(match["index"]), self.sensor_dims[match["name"]])

    def test_gpio_directions_and_sources(self):
        gpios = {item.attrib["name"]: item for item in self.control.findall("gpio")}
        self.assertEqual(set(gpios), {"input_only", "output_only", "bidirectional"})
        self.assertTrue(interface_names(gpios["input_only"], "state_interface"))
        self.assertFalse(interface_names(gpios["input_only"], "command_interface"))
        self.assertFalse(interface_names(gpios["output_only"], "state_interface"))
        self.assertTrue(interface_names(gpios["output_only"], "command_interface"))
        self.assertTrue(interface_names(gpios["bidirectional"], "state_interface"))
        self.assertTrue(interface_names(gpios["bidirectional"], "command_interface"))

        for component in gpios.values():
            component_params = params(component)
            for prefix, tag in (
                ("state", "state_interface"), ("command", "command_interface")
            ):
                for interface in interface_names(component, tag):
                    mapping = component_params[f"{prefix}.{interface}"]
                    match = MAPPING_RE.fullmatch(mapping)
                    self.assertIsNotNone(match, mapping)
                    if match["kind"] == "sensor":
                        self.assertIn(match["name"], self.sensor_names)
                        self.assertIsNotNone(match["index"])
                    else:
                        self.assertIn(match["name"], self.actuator_names)
                        self.assertIsNone(match["index"])

    def test_explicit_joint_actuators_exist(self):
        joint = self.control.find("joint")
        joint_params = params(joint)
        for key in ("position_actuator", "velocity_actuator", "effort_actuator"):
            self.assertIn(joint_params[key], self.actuator_names)

    def test_invalid_fixture_covers_required_failures(self):
        root = ET.parse(XML_DIR / "test_ros2_control_invalid_mappings.urdf").getroot()
        names = {item.attrib["name"] for item in root.findall("ros2_control")}
        self.assertEqual(
            names,
            {
                "MissingSensorSource",
                "UnknownSensor",
                "SensorIndexOutOfRange",
                "IncompleteImuProfile",
                "MissingGpioMapping",
                "DuplicateActuatorOwner",
                "InvalidScale",
            },
        )

    def test_plugin_load_fixtures_are_isolated(self):
        expected = {
            "test_plugin_load_clock.xml": "MujocoRosUtils::ClockPublisher",
            "test_plugin_load_sensor.xml": "MujocoRosUtils::SensorPublisher",
            "test_plugin_load_actuator.xml": "MujocoRosUtils::ActuatorCommand",
        }
        for filename, plugin_name in expected.items():
            root = ET.parse(XML_DIR / filename).getroot()
            declarations = {
                item.attrib["plugin"] for item in root.findall("./extension/plugin")
            }
            self.assertEqual(declarations, {plugin_name}, filename)
            instances = {
                item.attrib["plugin"] for item in root.iter("plugin")
                if item.find("config") is not None
            }
            self.assertEqual(instances, {plugin_name}, filename)

    def test_plugins_install_only_to_package_owned_directory(self):
        helper = (ROOT / "cmake" / "MujocoRosUtilsPlugins.cmake").read_text()
        self.assertIn('"lib/${PROJECT_NAME}/plugins"', helper)
        self.assertNotIn('"${MUJOCO_BIN_DIR}/mujoco_plugin"', helper)
        self.assertNotIn('DESTINATION "$ENV{MUJOCO_PLUGIN_PATH}"', helper)

    def test_lidar_sample_uses_direct_ray_configuration(self):
        root = ET.parse(XML_DIR / "sample_mujoco_ros_utils_lidar.xml").getroot()
        lidar_plugins = [
            item for item in root.findall("./sensor/plugin")
            if item.attrib.get("plugin") == "MujocoRosUtils::LidarPublisher"
        ]
        self.assertEqual(len(lidar_plugins), 2)
        configurations = [plugin_config(item) for item in lidar_plugins]
        self.assertEqual(
            {configuration["scan_pattern"] for configuration in configurations},
            {"custom", "vlp16"},
        )
        for configuration in configurations:
            self.assertEqual(configuration["site_name"], "lidar_origin")
            self.assertEqual(configuration["body_exclude"], "lidar_body")
            self.assertNotIn("sensor_name_prefix", configuration)
            self.assertNotIn("sensor_name_prefix_list", configuration)
        self.assertFalse(root.findall("./sensor/rangefinder"))
        legacy_include = (XML_DIR / "lidar_vlp16_sensors.xml").read_text()
        self.assertNotIn("sensor_name_prefix", legacy_include)
        self.assertIn('key="scan_pattern"          value="vlp16"', legacy_include)


if __name__ == "__main__":
    unittest.main()
