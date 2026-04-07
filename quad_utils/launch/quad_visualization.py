from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare all launch arguments

    declared_arguments = [
        DeclareLaunchArgument(
            'live_plot', default_value='false', description="Launch Plot Juggler on Startup"),
        DeclareLaunchArgument(
            'dash', default_value='false', description="Launch the RQT Dashboard"),
        DeclareLaunchArgument(
            'rviz', default_value='true', description="Launch RViz"),
    ]

    # Launch PlotJuggler
    plot_juggler_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('live_plot')),
        actions=[
            Node(
                package='plotjuggler',
                executable='plotjuggler',
                name='plotjuggler',
                arguments=[
                    '--layout',
                    PathJoinSubstitution([
                        FindPackageShare('quad_utils'),
                        'config',
                        'plotjuggler_config.xml'
                    ])
                ],
                # output='screen'
            )
        ]
    )

    # Launch RQT Dashboard
    rqt_dash_group = GroupAction(
            condition=IfCondition(LaunchConfiguration('dash')),
            actions=[
                Node(
                    package='rqt_gui',
                    executable='rqt_gui',
                    name='rqt_dashboard',
                    arguments=[
                        '--perspective-file',
                        PathJoinSubstitution([
                            FindPackageShare('quad_utils'),
                            'config',
                            'dashboard.perspective'
                        ])
                    ],
                    # output='screen'
                )
            ]
        )

    # Launch Rviz Interface from Config File
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('quad_utils'),
        'rviz',
        'quad_viewer.rviz'
    ])
    
    rviz2_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                # output='screen',
                arguments=['-d', rviz_config_file]
            )
        ]
    )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     parameters=[{ 'use_sim_time': True }],
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    #     output='screen'
    # )

    return LaunchDescription(
        declared_arguments + [
        plot_juggler_group,
        rqt_dash_group,
        rviz2_node,
        # static_tf
        ]
    )