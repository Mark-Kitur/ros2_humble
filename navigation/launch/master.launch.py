import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    pkg_nav = get_package_share_directory('navigation')
    
    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='true',
        description='Use simulation time'
    ))

    # Launch simulation first
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'final_1.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Launch navigation with a 10-second delay to ensure simulation is ready
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'final_m.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    ld.add_action(simulation_launch)
    ld.add_action(TimerAction(period=10.0, actions=[navigation_launch]))  # ✅ Added delay
    
    return ld