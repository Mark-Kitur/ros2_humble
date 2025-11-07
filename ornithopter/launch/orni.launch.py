from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os, xacro


def generate_launch_description():
    pkg_name = get_package_share_directory("ornithopter")

    # URDF/Xacro and world paths
    xacro_file = os.path.join(pkg_name, "urdf", "ornithopter.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()
    control_file=os.path.join(pkg_name, "config", "ornithopter_controller.yaml")

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])]
        ),
        launch_arguments={
            "verbose": "true",
        }.items(),
    )

    spawn_entity = TimerAction(
        period=0.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "ornithopter",
                    "-topic", "/robot_description",
                    '-x', '0', '-y', '0', '-z', '0', '-Y', '0.0'
                ],
                output="screen",
            )
        ]
    )


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[control_file],
        output="screen"
    )

    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
        ]
    )

    wing_controller =TimerAction(
        period=6.0,
        actions=[
             Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wing_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
        ]
    )

    tail_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tail_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_state_pub)
    # ld.add_action(controller_manager)

    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(wing_controller)
    ld.add_action(tail_controller)

    return ld
