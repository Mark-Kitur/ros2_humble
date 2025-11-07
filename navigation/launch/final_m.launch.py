import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    pkg_nav = get_package_share_directory('navigation')

    map_file = os.path.join(pkg_nav, 'map','my_map.yaml')
    nav_params = os.path.join(pkg_nav,'config', 'nav.yaml')
    amcl_params = os.path.join(pkg_nav, 'config','m_amcl.yaml')

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='true',
        description='Flag to enable use_sim_time'
    ))

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': map_file,
             'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable='amcl',
        name='amcl',
        parameters=[
            amcl_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    planner_ = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # REMOVED: bt_navigator and waypoint_follower (not needed for basic navigation)
    
    # Simplified lifecycle manager
    life_cycle_m = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'map_server', 
                'amcl', 
                'planner_server', 
                'controller_server',
                'behavior_server'
                # REMOVED: 'bt_navigator', 'waypoint_follower'
            ]
        }],
        output='screen'
    )

    # Add nodes with proper timing
    ld.add_action(map_server)
    ld.add_action(TimerAction(period=2.0, actions=[amcl_node]))
    ld.add_action(TimerAction(period=3.0, actions=[planner_]))
    ld.add_action(TimerAction(period=3.0, actions=[controller]))
    ld.add_action(TimerAction(period=3.0, actions=[behavior]))
    ld.add_action(TimerAction(period=5.0, actions=[life_cycle_m]))

    return ld