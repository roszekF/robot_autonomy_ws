from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''
    1. Launch the TurtleBot3 simulation in Gazebo (Turtlebot3 + some world)
    2. Launch the map publisher node (/custom_map topic)
    3. Launch rviz configured to visualize the /custom_map topic
    4. Launch the mapping node
    '''
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # that does not really work, use export before running the launch file
    turtlebot_model = DeclareLaunchArgument(
        'TURTLEBOT3_MODEL', default_value='waffle',
        description='Specify the model type of TurtleBot3 (burger, waffle, waffle_pi)'
    )

    # tb3_gazebo_share_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
    path_to_my_turtlebot_gazebo = os.path.join(get_package_share_directory('my_turtlebot'), 'launch', 'turtlebot_gazebo.launch.py') 
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('project_demo_bringup'), 'rviz', 'config_demo.rviz'
    ])

    # gazebo_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([tb3_gazebo_share_dir]),
    #     launch_arguments={'TURTLEBOT3_MODEL': LaunchConfiguration('TURTLEBOT3_MODEL')}.items()
    # )
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_to_my_turtlebot_gazebo]),
        launch_arguments={'model': LaunchConfiguration('TURTLEBOT3_MODEL')}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_custom_map_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'custom_map', 'odom'],  # x, y, z, yaw, pitch, roll, frame_id, child_frame_id
        name='static_transform_publisher_map_to_odom',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_custom_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'custom_map', 'map'],  # x, y, z, yaw, pitch, roll, frame_id, child_frame_id
        name='static_transform_publisher_map_to_odom',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    map_publisher_node = Node(
        package='final_project',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    behavior_tree_node = Node(
        package='final_project',
        executable='bt_publisher',
        name='bt_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    behavior_tree_node_delay = TimerAction(
        period=10.0,  # Delay time in seconds
        actions=[
            behavior_tree_node
        ]
    )

    return LaunchDescription([
        turtlebot_model,
        gazebo_launch_file,
        rviz_node,
        static_tf_custom_map_to_map,
        static_tf_custom_map_to_odom,
        map_publisher_node,
        behavior_tree_node_delay
    ])
