# Author: Mokshagna Anurag
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    px4_dir = os.path.expanduser('~/PX4-Autopilot')

    # Environment variables for headless SITL
    env_vars = {
        'PX4_GZ_WORLD': 'corridor',
        'PX4_PARAM_NAV_DLL_ACT': '0',
        'PX4_PARAM_NAV_RCL_ACT': '0',
        'PX4_PARAM_COM_RCL_EXCEPT': '4',
        'PX4_PARAM_COM_ARM_CHK': '0'
    }

    set_domain = SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='42')
    set_partition = SetEnvironmentVariable(name='GZ_PARTITION', value='barkath_drone')

    # MicroXRCEAgent
    micro_xrce = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # PX4 SITL
    # NOTE: Using gz_x500_depth which is our modified model with lidar_2d_v2 included
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth'],
        cwd=px4_dir,
        additional_env=env_vars,
        output='screen'
    )

    # ROS-Gazebo Bridge
    # Topic names are stable because the SDF now has explicit <topic> overrides:
    #   - OakD-Lite model.sdf: <topic>depth_camera</topic>     → Gz topic = /depth_camera
    #   - lidar_2d_v2 model.sdf: <topic>lidar_2d_v2/scan</topic> → Gz topic = /lidar_2d_v2/scan
    # These short names are portable — they won't break if the world/model name changes.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/lidar_2d_v2/scan', '/scan'),
        ],
        output='screen'
    )

    # Mapping Node
    mapping_node = Node(
        package='obstacle_avoidance',
        executable='mapping_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Path Planner
    path_planner = Node(
        package='obstacle_avoidance',
        executable='path_planner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Depth Avoidance (main flight controller)
    depth_avoidance = Node(
        package='obstacle_avoidance',
        executable='depth_avoidance',
        output='screen',
        parameters=[{'flight_altitude': 2.0, 'use_sim_time': True}]  # 2m hover height inside corridor
    )

    # RViz2 with config
    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'drone.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static TF publishers
    # The lidar scan has frame_id='link' (from gz_frame_id in lidar_2d_v2/model.sdf).
    # RViz needs a full chain: link → base_link → odom → map
    # These are approximate - they place the lidar at the front-top of the drone body.
    tf_link_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.12', '0', '0.26', '0', '0', '0', 'base_link', 'link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    tf_base_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    tf_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Delay the start of ROS 2 nodes to allow PX4/Gazebo to fully boot
    # 12s delay ensures EKF2 is initialized and uXRCE-DDS bridge is handshaking
    delayed_nodes = TimerAction(
        period=12.0,
        actions=[
            bridge,
            tf_link_to_base,
            tf_base_to_odom,
            tf_odom_to_map,
            mapping_node,
            path_planner,
            depth_avoidance,
            rviz
        ]
    )

    return LaunchDescription([
        set_domain,
        set_partition,
        micro_xrce,
        px4_sitl,
        delayed_nodes
    ])
