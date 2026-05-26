# Copyright (c) 2023 Open Navigation LLC
# Licensed under the Apache License, Version 2.0

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'titan_nav'

    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'route.yaml'
    )

    map_file = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'warehouse_map.yaml'
    )

    graph_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'warehouse_graph_fixed.geojson'
    )

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'route_server',
        'bt_navigator',
        'velocity_smoother',
    ]


    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Nav2 container
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[params_file],
        remappings=remappings
    )

    load_composable_nodes = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[

            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params_file],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav')
                ],
            ),

            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params_file],
                remappings=remappings,
            ),

            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[params_file],
                remappings=remappings,
            ),

            ComposableNode(
                package='nav2_route',
                plugin='nav2_route::RouteServer',
                name='route_server',
                parameters=[params_file, {'graph_file': graph_file}],
                remappings=remappings,
            ),

            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[params_file],
                remappings=remappings,
            ),

            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[params_file],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')
                ],
            ),

            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[{
                    'yaml_filename': map_file,
                    'use_sim_time': True
                }],
                remappings=remappings,
            ),

            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[params_file],
                remappings=remappings,
            ),

            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': lifecycle_nodes
                }],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(nav2_container)
    ld.add_action(load_composable_nodes)

    return ld
