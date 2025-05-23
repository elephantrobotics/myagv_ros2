import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    nav_dir = get_package_share_directory('myagv_navigation2')
    nav_launch_dir = os.path.join(nav_dir,'launch')

    nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir,'navigation2_active.launch.py')),
    )

    send_tfodom = launch_ros.actions.Node(
            package='myagv_multi', 
            executable='send_tfodom.py', 
            name='send_tfodom',
            output='screen'
    )

    return LaunchDescription([
        nav,send_tfodom
    ])

