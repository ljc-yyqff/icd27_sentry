# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("icd2027sentry_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    # The CAD-derived maps are already in Gazebo world coordinates. Point-LIO's
    # odom frame starts at the robot, whose Gazebo spawn is (-4.8, 3.5).
    cad_world = ["'", world, "' in ('rmul_2026', 'rmul_2027')"]
    use_sim_static_tf = PythonExpression(cad_world)
    map_to_odom_x = PythonExpression(["-4.8 if ", *cad_world, " else 0.0"])
    map_to_odom_y = PythonExpression(["3.5 if ", *cad_world, " else 0.0"])
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_ground_truth = LaunchConfiguration("use_sim_ground_truth")

    tunnel_file = PythonExpression([
        "'", os.path.join(bringup_dir, "config", "simulation", "rmul_2027_tunnels.yaml"),
        "' if '", world, "' == 'rmul_2027' else ''",
    ])
    pose_tree = PythonExpression([
        "'", os.path.join(bringup_dir, "behavior_trees", "navigate_to_pose_with_tunnels.xml"),
        "' if '", world, "' == 'rmul_2027' else '",
        os.path.join(bringup_dir, "behavior_trees", "navigate_to_pose_w_replanning_and_recovery.xml"),
        "'",
    ])
    poses_tree = PythonExpression([
        "'", os.path.join(bringup_dir, "behavior_trees", "navigate_through_poses_with_tunnels.xml"),
        "' if '", world, "' == 'rmul_2027' else '",
        os.path.join(bringup_dir, "behavior_trees", "navigate_through_poses_w_replanning_and_recovery.xml"),
        "'",
    ])
    # Rewrite the un-namespaced source passed down to bringup as well as the
    # pointcloud helper. Other worlds keep their original behavior trees.
    params_file = RewrittenYaml(
        source_file=params_file, root_key="",
        param_rewrites={
            "tunnel_config_file": tunnel_file,
            "default_nav_to_pose_bt_xml": pose_tree,
            "default_nav_through_poses_bt_xml": poses_tree,
            "planner_server.ros__parameters.GridBased.plugin": PythonExpression([
                "'icd2027sentry_nav2_plugins/FootprintGridPlanner' if '", world,
                "' == 'rmul_2027' else 'nav2_theta_star_planner/ThetaStarPlanner'"
            ]),
            "robot_radius": PythonExpression([
                "0.36 if '", world, "' == 'rmul_2027' else 0.2"
            ]),
            # Theta* integrates the squared inflation cost at every grid step.
            # The legacy weight 10 makes the entire narrow tunnel approach much
            # more expensive than circling the platform, even when it is free.
            # Keep all lethal cells; only balance cost against metric distance.
            "w_traversal_cost": PythonExpression([
                "0.5 if '", world, "' == 'rmul_2027' else 10.0"
            ]),
            "controller_server.ros__parameters.FollowPath.use_footprint_collision": PythonExpression([
                "'", world, "' == 'rmul_2027'"
            ]),
            "controller_server.ros__parameters.FollowPath.collision_regulated_lookahead": PythonExpression([
                "'", world, "' == 'rmul_2027'"
            ]),
            # At 1.8 m/s the complete one-second physical sweep plus sensor
            # offset and rolling-map latency must fit inside the local grid.
            "local_costmap.local_costmap.ros__parameters.width": PythonExpression([
                "7 if '", world, "' == 'rmul_2027' else 5"
            ]),
            "local_costmap.local_costmap.ros__parameters.height": PythonExpression([
                "7 if '", world, "' == 'rmul_2027' else 5"
            ]),
        }, convert_types=True,
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom)",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2027",
        description="Select world: rmul_2024, rmuc_2024, rmul_2025, rmuc_2025, rmul_2026 or rmul_2027",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "simulation", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "simulation", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    start_velodyne_convert_tool = Node(
        condition=UnlessCondition(use_sim_ground_truth),
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
    )

    # Keep the 2026 simulation pose aligned with Gazebo even if Point-LIO
    # temporarily cannot produce odometry from its simulated lidar stream.
    gazebo_ground_truth_tf = Node(
        package="icd2027sentry_nav_bringup",
        executable="gazebo_ground_truth_tf.py",
        name="gazebo_ground_truth_tf",
        namespace=namespace,
        condition=IfCondition(use_sim_static_tf),
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"spawn_x": map_to_odom_x},
            {"spawn_y": map_to_odom_y},
            {"authoritative": use_sim_ground_truth},
            {"use_full_ground_truth": use_sim_ground_truth},
            {"publish_registered_scan": use_sim_ground_truth},
            # ICD body / wheel envelope, in the timestamped actual chassis frame.
            # This is only consumed when registered-scan publishing is enabled.
            {"self_filter_bounds": [-0.36, 0.36, -0.36, 0.36, -0.08, 0.22]},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "use_sim_static_tf": use_sim_static_tf,
            "map_to_odom_x": map_to_odom_x,
            "map_to_odom_y": map_to_odom_y,
            "map": map_yaml_file,
            "use_tunnel_guard": "True",
            "use_sim_ground_truth": use_sim_ground_truth,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
    )

    joy_teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "joy_teleop_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "joy_config_file": params_file,
        }.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(DeclareLaunchArgument(
        "use_sim_ground_truth",
        default_value=PythonExpression(["'", world, "' == 'rmul_2027'"]),
        description="Use consistent Gazebo pose and raw lidar transforms in the narrow CAD field",
    ))
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(gazebo_ground_truth_tf)
    ld.add_action(bringup_cmd)
    ld.add_action(joy_teleop_cmd)
    ld.add_action(rviz_cmd)

    return ld
