#!/usr/bin/env python3
"""Offline launch/parameter regression, including the actual ROS C YAML parser.

Run after sourcing /opt/ros/humble/setup.bash and this workspace's
install/setup.bash. This evaluates launch substitutions/conditions and creates
LoadNode requests, but never executes a Node action, initializes rclpy/DDS,
starts Gazebo, or connects to a running ROS graph.
"""

import ctypes
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
import warnings
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "icd2027sentry_nav_bringup/launch"
NAMESPACES = ("red_standard_robot1", "blue_standard_robot1", "team/test_robot", "")


class Allocator(ctypes.Structure):
    # Exact rcutils_allocator_t layout from rcutils/allocator.h. Function
    # pointers are opaque to Python; the C implementation invokes them.
    _fields_ = [
        (name, ctypes.c_void_p)
        for name in ("allocate", "deallocate", "reallocate", "zero_allocate", "state")
    ]


class ErrorString(ctypes.Structure):
    _fields_ = [("message", ctypes.c_char * 1024)]


class RosParameterParser:
    """Invoke rcl_parse_yaml_file directly without creating a ROS context."""

    def __init__(self):
        self.rcutils = ctypes.CDLL("librcutils.so")
        self.library = ctypes.CDLL("librcl_yaml_param_parser.so")
        self.rcutils.rcutils_get_default_allocator.argtypes = []
        self.rcutils.rcutils_get_default_allocator.restype = Allocator
        self.rcutils.rcutils_get_error_string.argtypes = []
        self.rcutils.rcutils_get_error_string.restype = ErrorString
        self.rcutils.rcutils_reset_error.argtypes = []
        self.rcutils.rcutils_reset_error.restype = None
        self.library.rcl_yaml_node_struct_init.argtypes = [Allocator]
        self.library.rcl_yaml_node_struct_init.restype = ctypes.c_void_p
        self.library.rcl_yaml_node_struct_fini.argtypes = [ctypes.c_void_p]
        self.library.rcl_yaml_node_struct_fini.restype = None
        self.library.rcl_parse_yaml_file.argtypes = [ctypes.c_char_p, ctypes.c_void_p]
        self.library.rcl_parse_yaml_file.restype = ctypes.c_bool

    def parse(self, path):
        self.rcutils.rcutils_reset_error()
        params = self.library.rcl_yaml_node_struct_init(
            self.rcutils.rcutils_get_default_allocator()
        )
        if not params:
            raise MemoryError("rcl_yaml_node_struct_init failed")
        try:
            success = self.library.rcl_parse_yaml_file(os.fsencode(path), params)
            error = self.rcutils.rcutils_get_error_string().message.decode(
                errors="replace"
            )
            return success, error
        finally:
            self.library.rcl_yaml_node_struct_fini(params)
            self.rcutils.rcutils_reset_error()


def load_launch(name):
    path = LAUNCH / name
    spec = importlib.util.spec_from_file_location(
        "test_" + name.replace(".", "_"), path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def evaluate(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def make_context(description, inherited=None, **overrides):
    context = LaunchContext()
    if inherited is not None:
        context.launch_configurations.update(inherited.launch_configurations)
    context.launch_configurations.update(overrides)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def is_enabled(entity, context):
    return entity.condition is None or entity.condition.evaluate(context)


def node_name(node, context):
    return evaluate(context, getattr(node, "_Node__node_name", []))


def include_arguments(description, context):
    for entity in description.entities:
        entities = (
            entity.get_sub_entities() if isinstance(entity, GroupAction) else [entity]
        )
        for child in entities:
            if isinstance(child, IncludeLaunchDescription):
                yield {
                    key: evaluate(context, value)
                    for key, value in child.launch_arguments
                }


def navigation_context(top_name, namespace, composition, world=None):
    description = load_launch(top_name)
    overrides = {
        "namespace": namespace,
        "use_composition": str(composition),
        "use_rviz": "False",
    }
    if world is not None:
        overrides["world"] = world
    context = make_context(description, **overrides)
    bringup_args = next(
        args
        for args in include_arguments(description, context)
        if "params_file" in args
    )
    bringup = load_launch("bringup_launch.py")
    bringup_context = make_context(bringup, context, **bringup_args)
    args = list(include_arguments(bringup, bringup_context))
    navigation_args = next(
        entry for entry in args if "container_name" in entry and "map" not in entry
    )
    localization_args = next(
        entry for entry in args if "container_name" in entry and "map" in entry
    )
    navigation = load_launch("navigation_launch.py")
    final_context = make_context(navigation, bringup_context, **navigation_args)
    # Emulate only PushRosNamespace's scope value. Executing the real
    # GroupAction/Node/LoadComposableNodes actions would start ROS processes.
    final_context.launch_configurations["ros_namespace"] = (
        "/" + namespace.strip("/") if namespace else "/"
    )
    localization = load_launch("localization_launch.py")
    localization_context = make_context(
        localization, bringup_context, **localization_args
    )
    return (
        description,
        context,
        navigation,
        final_context,
        localization,
        localization_context,
    )


class TunnelLaunchConfigTest(unittest.TestCase):
    def test_icd_omni_wheel_contact_has_low_roller_resistance(self):
        import xml.etree.ElementTree as ET

        source = Path(__file__).resolve().parents[1] / (
            "external/icd2027sentry_gz_resources/resource/models/icd_sentry/"
            "chassis_wheel.def.xmacro"
        )
        root = ET.parse(source).getroot()
        contacts = root.findall(".//surface/friction/ode")
        self.assertEqual(len(contacts), 2)
        for contact in contacts:
            self.assertEqual(float(contact.findtext("mu")), 0.01)
            self.assertEqual(float(contact.findtext("mu2")), 0.01)
        for geometry in root.findall(".//collision/xmacro_block"):
            self.assertEqual(geometry.attrib["name"], "geometry_cylinder")
            self.assertEqual(float(geometry.attrib["r"]), 0.0758)
        servo = ET.parse(
            Path(__file__).resolve().parents[1]
            / (
                "external/icd2027sentry_robot_description/resource/xmacro/"
                "simulation_robot.sdf.xmacro"
            )
        ).getroot()
        lift = next(
            p
            for p in servo.findall(".//plugin")
            if p.findtext("joint_name") == "chassis_lift_joint"
        )
        self.assertEqual(float(lift.findtext("i_gain")), 20000.0)
        self.assertEqual(float(lift.findtext("cmd_max")), 2000.0)
        drive = next(p for p in servo.findall(".//plugin")
                     if p.attrib.get("filename") == "MecanumDrive2")
        self.assertEqual(drive.findtext("publish_full_ground_truth"), "true")
        self.assertEqual(float(drive.findtext("x_velocity_gain")), 500.0)
        self.assertEqual(float(drive.findtext("x_force_limit")), 200.0)

    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="rmul2027_launch_config_")
        cls.addClassCleanup(cls.directory.cleanup)
        # Keep every launch-generated rewritten YAML/log inside this test's
        # temporary directory, with no writes under ~/.ros or the workspace.
        cls.previous_tempdir = tempfile.tempdir
        tempfile.tempdir = cls.directory.name
        cls.addClassCleanup(setattr, tempfile, "tempdir", cls.previous_tempdir)
        os.environ.setdefault("ROS_LOG_DIR", str(Path(cls.directory.name) / "logs"))
        # Humble's RewrittenYaml uses yaml.safe_load(open(...)) without a
        # context manager. This upstream file-handle warning is unrelated to
        # parameter validity and otherwise repeats for every matrix case.
        warnings.filterwarnings(
            "ignore",
            category=ResourceWarning,
            module=r"nav2_common\.launch\.rewritten_yaml",
        )
        cls.parser = RosParameterParser()

    def assert_ros_yaml(self, path):
        text = Path(path).read_text()
        aliases = [
            event for event in yaml.parse(text) if isinstance(event, yaml.AliasEvent)
        ]
        self.assertEqual(aliases, [], f"ROS parameter YAML contains aliases: {path}")
        accepted, error = self.parser.parse(path)
        self.assertTrue(accepted, f"ROS C YAML parser rejected {path}: {error}")
        return yaml.safe_load(text)

    def active_navigation_nodes(self, description, context):
        names = []
        for entity in description.entities:
            if not is_enabled(entity, context):
                continue
            if isinstance(entity, LoadComposableNodes):
                for (
                    component
                ) in entity._LoadComposableNodes__composable_node_descriptions:
                    # This Humble LoadComposableNodes implementation ignores
                    # ComposableNode.condition, so all conditional selection
                    # must happen at the parent loading action.
                    condition = component.condition
                    self.assertIsNone(condition() if callable(condition) else condition)
                    request = get_composable_node_load_request(component, context)
                    names.append(request.node_name)
            elif isinstance(entity, GroupAction):
                names.extend(
                    node_name(child, context)
                    for child in entity.get_sub_entities()
                    if isinstance(child, Node) and is_enabled(child, context)
                )
            elif isinstance(entity, Node):
                names.append(node_name(entity, context))
        return names

    def test_ros_parser_rejects_alias_that_pyyaml_accepts(self):
        path = Path(self.directory.name) / "alias_fixture.yaml"
        path.write_text(
            "node:\n  ros__parameters:\n    first: &shared [1.0, 2.0]\n    second: *shared\n"
        )
        self.assertEqual(
            yaml.safe_load(path.read_text())["node"]["ros__parameters"]["second"],
            [1.0, 2.0],
        )
        accepted, error = self.parser.parse(path)
        self.assertFalse(accepted)
        self.assertIn("alias", error.lower())

    def test_simulation_matrix_final_ros_params_and_conditions(self):
        for world in ("rmul_2027", "rmul_2026"):
            for composition in (True, False):
                for namespace in NAMESPACES:
                    with self.subTest(
                        world=world, composition=composition, namespace=namespace
                    ):
                        (
                            top,
                            top_context,
                            nav,
                            context,
                            loc,
                            loc_context,
                        ) = navigation_context(
                            "icd2027sentry_navigation_simulation.launch.py",
                            namespace,
                            composition,
                            world,
                        )
                        expected_gt = world == "rmul_2027"
                        self.assertEqual(
                            context.launch_configurations["use_sim_ground_truth"],
                            str(expected_gt),
                        )
                        self.assertEqual(
                            context.launch_configurations["use_tunnel_guard"], "True"
                        )
                        helper = next(
                            e
                            for e in top.entities
                            if isinstance(e, Node)
                            and node_name(e, top_context) == "gazebo_ground_truth_tf"
                        )
                        helper_params = evaluate_parameters(
                            top_context, helper._Node__parameters
                        )
                        self.assertEqual(
                            list(
                                next(
                                    p["self_filter_bounds"]
                                    for p in helper_params
                                    if isinstance(p, dict) and "self_filter_bounds" in p
                                )
                            ),
                            [-0.36, 0.36, -0.36, 0.36, -0.08, 0.22],
                        )
                        for parameter in ("authoritative", "publish_registered_scan"):
                            self.assertEqual(
                                next(
                                    p[parameter]
                                    for p in helper_params
                                    if isinstance(p, dict) and parameter in p
                                ),
                                expected_gt,
                            )
                        converter = next(
                            e
                            for e in top.entities
                            if isinstance(e, Node)
                            and node_name(e, top_context) == "ign_sim_pointcloud_tool"
                        )
                        self.assertEqual(
                            is_enabled(converter, top_context), not expected_gt
                        )
                        point_lio = next(
                            e
                            for e in loc.entities
                            if isinstance(e, Node)
                            and node_name(e, loc_context) == "point_lio"
                        )
                        self.assertEqual(
                            is_enabled(point_lio, loc_context), not expected_gt
                        )
                        active = self.active_navigation_nodes(nav, context)
                        self.assertEqual(active.count("fake_vel_transform"), 1)
                        self.assertEqual(active.count("tunnel_guard"), 1)
                        for legacy in ("loam_interface", "sensor_scan_generation"):
                            self.assertEqual(legacy in active, not expected_gt)
                        guard = next(
                            e
                            for e in nav.entities
                            if isinstance(e, Node)
                            and node_name(e, context) == "tunnel_guard"
                        )
                        final_path = guard._Node__parameters[0].evaluate(context)
                        parsed = self.assert_ros_yaml(final_path)
                        params = parsed[namespace] if namespace else parsed
                        bt = params["bt_navigator"]["ros__parameters"]
                        controller = params["controller_server"]["ros__parameters"]
                        approach, follow = (
                            controller["TunnelApproach"],
                            controller["TunnelFollowPath"],
                        )
                        self.assertIn(
                            "TunnelApproach", controller["controller_plugins"]
                        )
                        self.assertIn(
                            "TunnelFollowPath", controller["controller_plugins"]
                        )
                        self.assertFalse(approach["tunnel_mode"])
                        self.assertTrue(approach["use_footprint_collision"])
                        self.assertEqual(
                            approach["collision_footprint"],
                            follow["collision_footprint"],
                        )
                        self.assertTrue(follow["tunnel_mode"])
                        self.assertEqual(approach["plugin"], follow["plugin"])
                        self.assertEqual(approach["v_linear_max"], 1.8)
                        self.assertEqual(follow["v_linear_max"], 1.8)
                        ordinary = controller["FollowPath"]
                        self.assertEqual(ordinary["use_footprint_collision"], expected_gt)
                        self.assertEqual(ordinary["collision_regulated_lookahead"], expected_gt)
                        self.assertEqual(ordinary["collision_prediction_time"], 1.0)
                        for follower in (approach, follow):
                            self.assertTrue(follower["linear_approach_scaling"])
                        planner = params["planner_server"]["ros__parameters"]["GridBased"]
                        self.assertEqual(planner["plugin"],
                            "icd2027sentry_nav2_plugins/FootprintGridPlanner" if expected_gt
                            else "nav2_theta_star_planner/ThetaStarPlanner")
                        self.assertEqual(
                            params["fake_vel_transform"]["ros__parameters"][
                                "output_cmd_vel_topic"
                            ],
                            "cmd_vel_unguarded",
                        )
                        tunnel_paths = [
                            bt["tunnel_config_file"],
                            params["tunnel_guard"]["ros__parameters"][
                                "tunnel_config_file"
                            ],
                        ]
                        for scope, mode in (
                            ("global_costmap", "global"),
                            ("local_costmap", "local"),
                        ):
                            costmap = params[scope][scope]["ros__parameters"]
                            if mode == "local":
                                self.assertTrue(costmap["always_send_full_costmap"])
                                self.assertEqual(costmap["width"], 7 if expected_gt else 5)
                                self.assertEqual(costmap["height"], 7 if expected_gt else 5)
                            self.assertEqual(
                                costmap["robot_radius"], 0.36 if expected_gt else 0.2
                            )
                            layer = costmap["intensity_voxel_layer"]
                            self.assertEqual(layer["tunnel_mode"], mode)
                            tunnel_paths.append(layer["tunnel_config_file"])
                        expected_joint_topic = (
                            "/" + namespace.strip("/") if namespace else ""
                        ) + "/joint_states"
                        self.assertEqual(
                            params["local_costmap"]["local_costmap"]["ros__parameters"][
                                "intensity_voxel_layer"
                            ]["tunnel_joint_states_topic"],
                            expected_joint_topic,
                        )
                        self.assertEqual(len(set(tunnel_paths)), 1)
                        self.assertTrue(
                            Path(tunnel_paths[0]).is_file()
                            if expected_gt
                            else tunnel_paths[0] == ""
                        )
                        for key in (
                            "default_nav_to_pose_bt_xml",
                            "default_nav_through_poses_bt_xml",
                        ):
                            path = Path(bt[key])
                            self.assertTrue(path.is_file())
                            tree = ET.parse(path)
                            self.assertEqual(
                                bool(tree.findall(".//TunnelNavigation")), expected_gt
                            )

    def test_reality_has_no_guard_output_competing_with_cmd_vel(self):
        for composition in (True, False):
            for namespace in NAMESPACES:
                with self.subTest(composition=composition, namespace=namespace):
                    _, _, nav, context, _, _ = navigation_context(
                        "rm_navigation_reality_launch.py", namespace, composition
                    )
                    self.assertEqual(
                        context.launch_configurations["use_tunnel_guard"], "False"
                    )
                    self.assertEqual(
                        context.launch_configurations["use_sim_ground_truth"], "False"
                    )
                    self.assertNotIn(
                        "tunnel_guard", self.active_navigation_nodes(nav, context)
                    )
                    guard = next(
                        e
                        for e in nav.entities
                        if isinstance(e, Node)
                        and node_name(e, context) == "tunnel_guard"
                    )
                    self.assertFalse(is_enabled(guard, context))
                    final_path = guard._Node__parameters[0].evaluate(context)
                    parsed = self.assert_ros_yaml(final_path)
                    params = parsed[namespace] if namespace else parsed
                    self.assertEqual(
                        params["fake_vel_transform"]["ros__parameters"][
                            "output_cmd_vel_topic"
                        ],
                        "cmd_vel",
                    )


if __name__ == "__main__":
    unittest.main()
