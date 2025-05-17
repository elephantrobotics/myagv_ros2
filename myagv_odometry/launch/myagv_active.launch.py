import os
from launch import LaunchDescription
from launch_ros.actions import Node,PushRosNamespace
from launch.substitutions import Command,LaunchConfiguration,PythonExpression
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file = os.path.join(
        get_package_share_directory('myagv_description'),
        'urdf',
        'myAGV.urdf'
    )

    robot_description_content = Command([
        'xacro ',
        urdf_file,
        ' namespace:=',
        PythonExpression(['"', namespace, '" + "/" if "', namespace, '" != "" else ""']),
    ])

    ekf_config_file = os.path.join(
        get_package_share_directory('myagv_odometry'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for nodes'),

        PushRosNamespace(namespace),

        Node(
            package='myagv_odometry',
            executable='myagv_odometry_node',
            name='myagv_odometry_node',
            parameters=[
                {'namespace': namespace}],
            remappings=[
                ('cmd_vel', '/cmd_vel')],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[
                {'use_sim_time': False}],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description_content},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name = 'ekf_node',
            output='screen',
            parameters=[
                ekf_config_file,
                {
                    'odom0':'odom',
                    'imu0': 'imu',
                    'odom_frame': 'odom',
                    'base_link_frame': 'base_footprint',
                    'world_frame': 'odom',
                    'imu0_config': [
                        False, False, False,
                        False, False, False,
                        False, False, False,
                        False, False, True,
                        False, False, False
                    ],
                    'odom0_config': [
                        False, False, False,
                        False, False, False,
                        True, True, False,
                        False, False, True,
                        False, False, False
                    ]
                }
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),'launch'),
                '/ydlidar_launch.py']),
            launch_arguments={'namespace': namespace}.items(),
        )
    ])
