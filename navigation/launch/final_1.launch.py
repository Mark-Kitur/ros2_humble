import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = os.path.join(get_package_share_directory('navigation'))
    xacro_file = os.path.join(pkg_path,'urdf','nav2_.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    efk_file =os.path.join(pkg_path,'config','ekf.yaml')
    bridg_file= os.path.join(pkg_path,'config','bridge_config.yaml')

    ld.add_action(DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Flag to enable use_sim_time'))

    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}],
    )
    
    robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description':robot_description},
                    {'use_sim_time':LaunchConfiguration('use_sim_time')}],
        output ='screen'
    )
    
    # spawn
    spawn_node =Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            "-entity","nav2",
            "-x","0.05",
            "-y","0.1",
            "-z","0",
            "-Y","0",
            "-topic","/robot_description"
        ],
        output ='screen'
    )
    
    world_path= os.path.join(pkg_path,'worlds','maze.sdf')
    gazebo_node= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                   'launch','gazebo.launch.py'])]
        ),
        launch_arguments={
            "world":world_path,
            'verbose':'true'
        }.items()
    )
    
    rviz_node= Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[efk_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # FIXED: Proper bridge configuration
    # ✅ CORRECT: Use consistent bracket types
    ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry',      # Changed @ to ]
            '/scan@sensor_msgs/msg/LaserScan]gz.msgs.LaserScan', # Changed @ to ]
            '/clock@rosgraph_msgs/msg/Clock]gz.msgs.Clock'       # Changed [ to ]
        ],
        output='screen'
    )
        

    ld.add_action(gazebo_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_node)
    #ld.add_action(ros_bridge)
    ld.add_action(robot_localization_node)
    ld.add_action(joint_state_publisher_node)
    #ld.add_action(rviz_node)
    return ld