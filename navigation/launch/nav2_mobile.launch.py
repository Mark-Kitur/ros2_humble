import xacro
import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory,get_package_share_path
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter



def generate_launch_description():
    
    ld = LaunchDescription()

    # set use_sim_time = true
    # use_sim_time = SetParameter(name='use_sim_time',value=True)
    # ld.add_action(use_sim_time)
    # parse the xacro to urdf
    pkg_path = os.path.join(get_package_share_directory('navigation'))
    xacro_file = os.path.join(pkg_path,'urdf','nav2_.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node =Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description},
                    {'use_sim_time':True}]
    )

    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time':True}],
        output='screen'
    )
    rviz_node =Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d',os.path.join(get_package_share_path('navigation'),'rviz',robot_description)]
        output='screen'
    )

    # spawn
    spawn_node =Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            "-entity","nav2",
            "-x","0.3",
            "-y","0",
            "-z","0",
            "-Y","0",
            "-topic","/robot_description"
        ],
        output ='screen'
    )

    pkg_share = FindPackageShare('navigation').find('navigation')
    world_path = os.path.join(pkg_share,'worlds','maze.sdf')

    gazebo_node=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                                   'launch','gazebo.launch.py'])]
        ),
        launch_arguments={
            'verbose':'true',
            'pause':'false',
            'world':world_path
        }.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        remappings=[
            ('/odom', '/odom'),
            ('/tf', '/tf'),
        ],
        output='screen'
    )
    base=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        )
    
    map_to_odom = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='map_to_odom_broadcaster',
                    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
                )
                        
    ld.add_action(map_to_odom)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_node)           # start Gazebo first
    ld.add_action(spawn_node)            # spawn robot after Gazebo is ready
    ld.add_action(bridge)                # start bridge after robot is spawned
    #ld.add_action(base)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)             # start RViz last



    
    return ld