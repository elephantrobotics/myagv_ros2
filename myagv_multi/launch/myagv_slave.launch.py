import os
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    base_dir = get_package_share_directory('myagv_odometry')
    base_launch_dir = os.path.join(base_dir,'launch')

    nav2_dir = get_package_share_directory('myagv_navigation2')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir,'launch')

    my_map_dir = os.path.join(nav2_dir, 'map')
    my_param_dir = os.path.join(nav2_dir, 'param')

    #<<**参数设置*********************************************************************
    #从机的期望跟随坐标  以主车为坐标原点
    slave_x = DeclareLaunchArgument('slave_x',default_value='0.0')  #slave_x为从车（跟随者）的期望x坐标，主车前方为正方向，单位：m
    slave_y = DeclareLaunchArgument('slave_y',default_value='0.8')   #slave_y为从车（跟随者）的期望y坐标，主车左方为正方向，单位：m
    # 编队模式选择  1：主车自转，从车围绕主车运动型    2：主车自转，从车原地自转型（从车模仿主车运动）
    multi_mode = DeclareLaunchArgument('multi_mode',default_value='2')
    #是否开启避障功能 True / False
    avoidance = DeclareLaunchArgument('avoidance', default_value='True')
    max_vel_x = DeclareLaunchArgument('max_vel_x',default_value='0.5')   # 从车最大线速度限制（绝对值）
    min_vel_x = DeclareLaunchArgument('min_vel_x',default_value='0.05')  # 从车最小线速度限制（绝对值）
    max_vel_theta = DeclareLaunchArgument('max_vel_theta',default_value='0.5')   #从车最大角速度限制（绝对值）
    min_vel_theta = DeclareLaunchArgument('min_vel_theta',default_value='0.05')  #从车最小角速度限制（绝对值）
    
    map_yaml_file = DeclareLaunchArgument('map_yaml_file', default_value=os.path.join(my_map_dir, 'map_0512.yaml'))
    params_file = DeclareLaunchArgument('params_file', default_value=os.path.join(my_param_dir, 'myagv.yaml'))
    avoidance_kv = DeclareLaunchArgument('avoidance_kv', default_value='0.4')  #修正从车线速度的参数，参数越大，障碍物对从车减速作用越强
    avoidance_kw = DeclareLaunchArgument('avoidance_kw', default_value='0.4')  #修正从车角速度的参数，参数越大，调节从车角度，使车头偏离障碍物的作用越强
    safe_distence = DeclareLaunchArgument('safe_distence', default_value='0.4')  #安全距离界限
    danger_distence = DeclareLaunchArgument('danger_distence', default_value='0.2')   #危险距离界限
    k_v = DeclareLaunchArgument('k_v', default_value='0.8')     #调节前后方向偏差时，k_v越大，线速度越大
    k_l = DeclareLaunchArgument('k_l', default_value='1.0')          #调节左右方向偏差时，k_l越大，角速度越大
    k_a = DeclareLaunchArgument('k_a', default_value='1.0')        #调节角度偏差时，k_a越大，角速度越大
    #**参数设置*********************************************************************>>

    #<<**定位*********************************************************************
    # 开启机器人底层相关节点和开启雷达
    robot_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_launch_dir,"myagv_active.launch.py")))

    #开启用于导航的自适应蒙特卡洛定位amcl
    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir,'localization_launch.py')),
                launch_arguments={'map': LaunchConfiguration('map_yaml_file'),
                                'use_sim_time': 'False',
                                'autostart': 'True',
                                'params_file': LaunchConfiguration('params_file'),
                                'use_composition': 'False',
                                'use_respawn': 'False',
                                'container_name': 'nav2_container'}.items())
    
    #多机编队的从机位姿初始化
    set_pose = launch_ros.actions.Node(
                package='myagv_multi', 
                executable='set_pose.py', 
                name='set_pose',
                parameters=[{'slave_x': LaunchConfiguration('slave_x'),
                    'slave_y': LaunchConfiguration('slave_y')
                }],
                output='screen')
    #**定位*********************************************************************>>

    #提取最近的障碍物距离信息
    laser_tracker = launch_ros.actions.Node(
                condition=IfCondition(LaunchConfiguration('avoidance')),
                package='myagv_multi', 
                executable='lasertracker', 
                name='lasertracker'
                )

    #小车避障
    multi_avoidance = launch_ros.actions.Node(
                condition=IfCondition(LaunchConfiguration('avoidance')),
                package='myagv_multi', 
                executable='multi_avoidance', 
                name='multi_avoidance',
                parameters=[{'max_vel_x': LaunchConfiguration('max_vel_x'),   #从车最大线速度限制（绝对值）
                    'min_vel_x': LaunchConfiguration('min_vel_x'),            #从车最小线速度限制（绝对值）
                    'max_vel_theta': LaunchConfiguration('max_vel_theta'),    #从车最大角速度限制（绝对值）
                    'min_vel_theta': LaunchConfiguration('min_vel_theta'),    #从车最小角速度限制（绝对值）
                    'avoidance_kv': LaunchConfiguration('avoidance_kv'),     #修正从车线速度的参数，参数越大，障碍物对从车减速作用越强
                    'avoidance_kw': LaunchConfiguration('avoidance_kw'),     #修正从车角速度的参数，参数越大，调节从车角度，使车头偏离障碍物的作用越强
                    'safe_distence': LaunchConfiguration('safe_distence'),   #安全距离界限
                    'danger_distence': LaunchConfiguration('danger_distence'),  #危险距离界限
                }],
                output='screen'
                )
    
    #从车接受主车位置与速度信息
    listen_tfodom = launch_ros.actions.Node(
                package='myagv_multi', 
                executable='listen_tfodom.py', 
                name='listen_tfodom',
                output='screen')
    
    #从机位置监听与从机速度控制节点
    slave_tf_listener_avoid = launch_ros.actions.Node(
                condition=IfCondition(LaunchConfiguration('avoidance')),
                package='myagv_multi', 
                executable='multi_slave', 
                name='multi_slave',
                parameters=[{'max_vel_x': LaunchConfiguration('max_vel_x'),
                    'min_vel_x': LaunchConfiguration('min_vel_x'),
                    'max_vel_theta': LaunchConfiguration('max_vel_theta'),
                    'min_vel_theta': LaunchConfiguration('min_vel_theta'),
                    'multi_mode': LaunchConfiguration('multi_mode'),    #编队模式选择
                    'slave_x': LaunchConfiguration('slave_x'),
                    'slave_y': LaunchConfiguration('slave_y'),
                    'avoidance': LaunchConfiguration('avoidance'),
                    'k_v': LaunchConfiguration('k_v'),                  #调节前后方向偏差时，k_v越大，线速度越大
                    'k_l': LaunchConfiguration('k_l'),                  #调节左右方向偏差时，k_l越大，角速度越大
                    'k_a': LaunchConfiguration('k_a'),                  #调节角度偏差时，k_a越大，角速度越大
                }],
                output='screen'
                )
    
    #从机位置监听与从机速度控制节点
    slave_tf_listener = launch_ros.actions.Node(
                condition=UnlessCondition(LaunchConfiguration('avoidance')),
                package='myagv_multi', 
                executable='multi_slave', 
                name='multi_slave',
                parameters=[{'max_vel_x': LaunchConfiguration('max_vel_x'),
                    'min_vel_x': LaunchConfiguration('min_vel_x'),
                    'max_vel_theta': LaunchConfiguration('max_vel_theta'),
                    'min_vel_theta': LaunchConfiguration('min_vel_theta'),
                    'multi_mode': LaunchConfiguration('multi_mode'),
                    'slave_x': LaunchConfiguration('slave_x'),
                    'slave_y': LaunchConfiguration('slave_y'),
                    'avoidance': LaunchConfiguration('avoidance'),
                    'k_v': LaunchConfiguration('k_v'),
                    'k_l': LaunchConfiguration('k_l'),
                    'k_a': LaunchConfiguration('k_a'),
                }],
                output='screen',
                remappings=[('/cmd_vel_ori', '/cmd_vel'),]   #不避障，则直接输出cmd_vel速度给从车，不经过multi_avoidance避障节点
                )

    return LaunchDescription([
        slave_x,
        slave_y,
        multi_mode,
        avoidance,
        max_vel_x,
        min_vel_x,
        max_vel_theta,
        min_vel_theta,
        map_yaml_file,
        params_file,
        avoidance_kv,
        avoidance_kw,
        safe_distence,
        danger_distence,
        k_v,
        k_l,
        k_a,

        # robot_odom,
        localization,
        set_pose,
        laser_tracker,
        multi_avoidance,
        listen_tfodom,
        slave_tf_listener,
        slave_tf_listener_avoid
    ])

