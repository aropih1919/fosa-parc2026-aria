import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _package_share(package_name: str) -> str:
    return get_package_share_directory(package_name)


def generate_launch_description():
    aria_bringup_share = _package_share('aria_bringup')
    aria_description_share = _package_share('aria_description')
    ros_gz_sim_share = _package_share('ros_gz_sim')
    nav2_bringup_share = _package_share('nav2_bringup')
    xacro_exe = 'xacro'

    world_file = LaunchConfiguration('world_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    navsat_params_file = LaunchConfiguration('navsat_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution(
            [aria_bringup_share, 'worlds', 'aria_world.sdf']
        ),
        description='Chemin du monde Gazebo ARIA',
    )
    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution(
            [aria_bringup_share, 'config', 'nav2_params.yaml']
        ),
        description='Fichier de paramètres Nav2',
    )
    declare_navsat_params_cmd = DeclareLaunchArgument(
        'navsat_params_file',
        default_value=PathJoinSubstitution(
            [aria_bringup_share, 'config', 'navsat_params.yaml']
        ),
        description='Fichier de paramètres navsat_transform_node',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Active le temps simulé',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Lance Nav2 automatiquement',
    )

    robot_xacro = PathJoinSubstitution(
        [aria_description_share, 'urdf', 'aria_robot.urdf.xacro']
    )
    robot_description_content = Command([xacro_exe, ' ', robot_xacro])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world_file],
        }.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
        ],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/zed/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[
            ('/zed/depth', '/camera/depth/image_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_to_odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_params_file,
            {
                'use_sim_time': use_sim_time,
                'use_odometry_yaw': True,
            },
        ],
        remappings=[
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odom'),
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'namespace': '',
            'log_level': 'info',
        }.items(),
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'aria_robot',
                    '-string', robot_description_content,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.20',
                    '-Y', '0.0',
                    '-allow_renaming', 'true',
                ],
            )
        ],
    )

    return LaunchDescription([
        declare_world_file_cmd,
        declare_nav2_params_cmd,
        declare_navsat_params_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        gz_sim_launch,
        robot_state_publisher_node,
        bridge_node,
        map_to_odom_static_tf,
        navsat_transform_node,
        nav2_launch,
        spawn_robot,
    ])
