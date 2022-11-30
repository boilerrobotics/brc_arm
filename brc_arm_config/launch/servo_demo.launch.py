import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "brc_arm_description", package_name="brc_arm_config"
    ).to_moveit_configs()

    ld = generate_demo_launch(moveit_config)    

    moveit_config_xacro = (
        MoveItConfigsBuilder("brc_arm_description", package_name="brc_arm_config")
        .robot_description(file_path="config/brc_arm_description.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    package_path = get_package_share_directory("brc_arm_config")
    absolute_file_path = os.path.join(package_path, "config/servo_simulated_config.yaml")
    try:
        with open(absolute_file_path, "r") as file:
            servo_yaml = yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
         return ld
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="robot_state_publisher",
            #     plugin="robot_state_publisher::RobotStatePublisher",
            #     name="robot_state_publisher",
            #     parameters=[moveit_config_xacro.robot_description],
            # ),
            # ComposableNode(
            #     package="tf2_ros",
            #     plugin="tf2_ros::StaticTransformBroadcasterNode",
            #     name="static_tf2_broadcaster",
            #     parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            # ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    ld.add_action(container)

    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[
                servo_params,
                moveit_config_xacro.robot_description,
                moveit_config_xacro.robot_description_semantic,
                moveit_config_xacro.robot_description_kinematics,
            ],
            output="screen",
        )
    )

    return ld
