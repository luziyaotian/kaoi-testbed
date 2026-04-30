"""
Unified testbed launch — dt_teleop_testbed.

One command to bring up everything:

    ros2 launch dt_teleop_testbed testbed.launch.py \\
         interface:=enx00e04c680123 \\
         robot_ip:=192.168.1.167 \\
         use_mock:=false

Parameters:
    use_mock          (bool=false)  If true, start mock_lite6 instead of the
                                    real xarm driver + bridge. For testing
                                    without hardware.
    interface         (str)         Ethernet interface for tc netem. Required
                                    unless skip_netem:=true.
    robot_ip          (str)         Lite 6 IP. Only used when use_mock=false.
    twin_mode         (str="sine")  Headless twin mode (sine/waypoints/replay).
    skip_netem        (bool=false)  If true, don't start network_conditioner.
    skip_logger       (bool=false)  If true, don't start aoi_logger.
    commander_kind    (str="planner") "planner" or "service".
    forward_rate_hz   (float=10.0)
    csv_path          (str="")      Output path for CSV; auto if empty.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    GroupAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("dt_teleop_testbed")
    profiles_yaml = os.path.join(
        pkg_share, "config", "network_profiles.yaml")

    use_mock = LaunchConfiguration("use_mock").perform(context).lower() == "true"
    skip_netem = LaunchConfiguration("skip_netem").perform(context).lower() == "true"
    skip_logger = LaunchConfiguration("skip_logger").perform(context).lower() == "true"

    iface = LaunchConfiguration("interface").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    twin_mode = LaunchConfiguration("twin_mode").perform(context)
    commander_kind = LaunchConfiguration("commander_kind").perform(context)
    forward_rate_hz = float(
        LaunchConfiguration("forward_rate_hz").perform(context))
    csv_path = LaunchConfiguration("csv_path").perform(context)
    use_sudo_tc = LaunchConfiguration("use_sudo_tc").perform(context).lower() == "true"
    use_unity = LaunchConfiguration("use_unity").perform(context).lower() == "true"
    unity_ip = LaunchConfiguration("unity_ip").perform(context)
    unity_port = LaunchConfiguration("unity_port").perform(context)

    use_scripted = LaunchConfiguration("use_scripted").perform(context).lower() == "true"

    nodes = []

    # ------------------------------------------------------------ #
    # Twin source selection:
    #   use_unity=true       -> ROS-TCP-Endpoint (Unity publishes)
    #   use_scripted=true    -> scripted_operator node
    #   else                 -> headless_twin (sine/waypoints/replay)
    # ------------------------------------------------------------ #
    if use_scripted:
        # Automated reproducible trajectory. Seeds from real robot pose
        # when real driver is used, else starts from zero.
        seed_topic = "/ufactory/joint_states" if not use_mock else ""
        nodes.append(Node(
            package="dt_teleop_testbed",
            executable="scripted_operator",
            name="scripted_operator",
            parameters=[{
                "topic": "/twin/joint_states",
                "rate_hz": 50.0,
                "j1_amplitude_rad": 0.5,
                "j1_period_s": 4.0,
                "j2_amplitude_rad": 0.3,
                "j2_period_s": 6.0,
                "initial_wait_s": 3.0,
                "seed_from_topic": seed_topic,
                "seed_wait_s": 5.0,
            }],
            output="screen",
        ))
    elif not use_unity:
        twin_params = {"mode": twin_mode, "rate_hz": 50.0}
        if twin_mode == "waypoints":
            twin_params["waypoints.file"] = os.path.join(
                pkg_share, "config", "waypoints_example.yaml")
        elif twin_mode == "replay":
            twin_params["replay.file"] = os.path.join(
                pkg_share, "config", "replay_example.csv")

        nodes.append(Node(
            package="dt_teleop_testbed",
            executable="headless_twin",
            name="headless_twin",
            parameters=[twin_params],
            output="screen",
        ))

    # ------------------------------------------------------------ #
    # ROS-TCP-Endpoint (only when Unity is the twin)
    # ------------------------------------------------------------ #
    if use_unity:
        nodes.append(Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="unity_tcp_endpoint",
            parameters=[{
                "ROS_IP": unity_ip,
                "ROS_TCP_PORT": int(unity_port),
            }],
            output="screen",
        ))

    # ------------------------------------------------------------ #
    # Robot — mock or real
    # ------------------------------------------------------------ #
    if use_mock:
        nodes.append(Node(
            package="dt_teleop_testbed",
            executable="mock_lite6",
            name="mock_lite6",
            parameters=[{
                "rate_hz": 50.0,
                "response_tau_s": 0.05,
                "cmd_topic": "/lite6_real/cmd_joint_state",
                "state_topic": "/lite6_real/joint_states",
            }],
            output="screen",
        ))
        # Simple relay from twin to mock, simulating the bridge path.
        # (Real mode uses the bridge instead.)
        nodes.append(Node(
            package="topic_tools",
            executable="relay",
            name="twin_to_mock_relay",
            arguments=[
                "/twin/joint_states",
                "/lite6_real/cmd_joint_state",
            ],
            output="screen",
        ))
    else:
        # Real robot: include the xarm driver launch, then our bridge.
        # The driver brings up /ufactory/* services and publishes
        # /ufactory/joint_states.
        xarm_api_share = FindPackageShare("xarm_api")
        driver_launch = PathJoinSubstitution([
            xarm_api_share, "launch", "lite6_driver.launch.py",
        ])
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([driver_launch]),
            launch_arguments={"robot_ip": robot_ip}.items(),
        ))

        nodes.append(Node(
            package="dt_teleop_testbed",
            executable="twin_to_lite6_bridge",
            name="twin_to_lite6_bridge",
            parameters=[{
                "commander_kind": commander_kind,
                "forward_rate_hz": forward_rate_hz,
                "twin_topic": "/twin/joint_states",
                "driver_state_topic": "/ufactory/joint_states",
                "real_state_topic": "/lite6_real/joint_states",
            }],
            output="screen",
        ))

    # ------------------------------------------------------------ #
    # Network conditioner
    # ------------------------------------------------------------ #
    if not skip_netem:
        if not iface:
            raise RuntimeError(
                "Parameter 'interface' is required unless skip_netem:=true")
        nodes.append(Node(
            package="dt_teleop_testbed",
            executable="network_conditioner",
            name="network_conditioner",
            parameters=[{
                "interface": iface,
                "profiles_file": profiles_yaml,
                "use_sudo": use_sudo_tc,
                "apply_on_startup": "ideal",
                "clear_on_shutdown": True,
            }],
            output="screen",
        ))

    # ------------------------------------------------------------ #
    # AoI logger
    # ------------------------------------------------------------ #
    if not skip_logger:
        nodes.append(Node(
            package="dt_teleop_testbed",
            executable="aoi_logger",
            name="aoi_logger",
            parameters=[{
                "twin_topic": "/twin/joint_states",
                "real_topic": "/lite6_real/joint_states",
                "profile_topic": "/network_profile",
                "log_rate_hz": 20.0,
                "csv_path": csv_path,
            }],
            output="screen",
        ))

    # ------------------------------------------------------------ #
    # Gripper bridge — only meaningful with real robot
    # ------------------------------------------------------------ #
    use_gripper = LaunchConfiguration("use_gripper").perform(context).lower() == "true"
    if use_gripper and not use_mock:
        nodes.append(Node(
            package="dt_teleop_testbed",
            executable="gripper_bridge",
            name="gripper_bridge",
            parameters=[{
                "command_topic": "/gripper/command",
                "state_topic": "/gripper/state",
                "open_service": "/ufactory/open_lite6_gripper",
                "close_service": "/ufactory/close_lite6_gripper",
                "dry_run": False,
            }],
            output="screen",
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_mock", default_value="false",
            description="Use mock robot instead of real Lite 6"),
        DeclareLaunchArgument("interface", default_value="",
            description="Ethernet interface for tc netem (required unless skip_netem:=true)"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.167",
            description="Lite 6 controller IP"),
        DeclareLaunchArgument("twin_mode", default_value="sine",
            description="Headless twin mode: sine | waypoints | replay"),
        DeclareLaunchArgument("skip_netem", default_value="false",
            description="Skip starting network_conditioner"),
        DeclareLaunchArgument("skip_logger", default_value="false",
            description="Skip starting aoi_logger"),
        DeclareLaunchArgument("commander_kind", default_value="planner",
            description="Bridge commander: planner | service"),
        DeclareLaunchArgument("forward_rate_hz", default_value="10.0",
            description="Bridge forward rate to robot"),
        DeclareLaunchArgument("csv_path", default_value="",
            description="AoI logger output path; auto if empty"),
        DeclareLaunchArgument("use_sudo_tc", default_value="false",
            description="True if tc requires sudo (false if setcap was used)"),
        DeclareLaunchArgument("use_unity", default_value="false",
            description="If true, start ROS-TCP-Endpoint and skip headless_twin (Unity is the twin)"),
        DeclareLaunchArgument("unity_ip", default_value="127.0.0.1",
            description="IP Unity connects from (127.0.0.1 for same-machine)"),
        DeclareLaunchArgument("unity_port", default_value="10000",
            description="TCP port for Unity endpoint"),
        DeclareLaunchArgument("use_gripper", default_value="false",
            description="Start gripper_bridge (Lite 6 gripper service). "
                        "Only effective with use_mock=false."),
        DeclareLaunchArgument("use_scripted", default_value="false",
            description="If true, start scripted_operator (automated, "
                        "reproducible trajectory) as the twin source. "
                        "Overrides use_unity and headless_twin."),

        OpaqueFunction(function=launch_setup),
    ])
