import os, xacro
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    pkg_path = get_package_share_directory('cart_pole')
    xacro_file = os.path.join(pkg_path, "urdf", "cart_pole_des.urdf.xacro")
    robot_des = xacro.process_file(xacro_file).toxml()

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_des}]
    )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={"verbose": "true"}.items(),
    )

    # --- Spawn robot into Gazebo ---
    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            '-entity', 'cartpole',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0', '-Y', '0.0'
        ],
        output='screen'
    )

    # --- Controller Spawners (instead of ros2 control load_controller) ---
    load_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    load_effort_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["effort_control", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    load_stick_effort_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["stick_effort_control", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    # --- Add everything to Launch Description ---
    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_effort_controller)
    ld.add_action(load_stick_effort_controller)

    return ld
