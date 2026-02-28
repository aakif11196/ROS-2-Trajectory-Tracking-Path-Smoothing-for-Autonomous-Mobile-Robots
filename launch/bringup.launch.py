import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_trajectory_tracking = get_package_share_directory('trajectory_tracking')
    pkg_turtlebot3_gazebo   = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim          = get_package_share_directory('ros_gz_sim')

    rviz_config_file = os.path.join(pkg_trajectory_tracking, 'rviz', 'tracking.rviz')

    gazebo_version_arg = DeclareLaunchArgument(
        'gazebo_version',
        default_value='classic',
        description="Choose gazebo version: 'classic' or 'ignition'"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for all nodes'
    )

    gazebo_version = LaunchConfiguration('gazebo_version')
    use_sim_time   = LaunchConfiguration('use_sim_time')

    classic_gazebo_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", gazebo_version, "' == 'classic'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_turtlebot3_gazebo, 'launch', 'empty_world.launch.py')
                )
            )
        ]
    )

    ignition_gazebo_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", gazebo_version, "' == 'ignition'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'turtlebot3', '-x', '0.0', '-y', '0.0', '-z', '0.1', '-topic', '/robot_description'],
                output='screen',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                ],
                output='screen',
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    trajectory_tracker_node = Node(
        package='trajectory_tracking',
        executable='trajectory_tracker_node',
        name='trajectory_tracker_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},

         # {'waypoints_x': [0.0, 2.0, 4.0, 4.0, 6.0, 8.0]}, 
          #{'waypoints_y': [0.0, 0.0, 0.0, 3.0, 4.5, 6.0]}

            # A perfect zig-zag path (snake-like movement)
            #{'waypoints_x': [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]},
           # {'waypoints_y': [0.0, 2.0, -2.0, 2.0, -2.0, 0.0]}

           # Irregular Zigzag / Roller-coaster Path
          {'waypoints_x': [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]},
          {'waypoints_y': [0.0, 1.0, -1.0, 0.0, 1.0, -1.0]}

         #{'waypoints_x': [0.0, 2.0, 3.5, 4.0, 4.0, 6.0, 8.0]},
         #{'waypoints_y': [0.0, 0.0, 0.0, 0.5, 3.0, 4.5, 6.0]}
        ]
    )

    delayed_tracker_node = TimerAction(
        period=10.0,
        actions=[trajectory_tracker_node]
    )

    return LaunchDescription([
        gazebo_version_arg,
        use_sim_time_arg,
        classic_gazebo_group,
        ignition_gazebo_group,
        rviz_node,
        delayed_tracker_node  
    ])