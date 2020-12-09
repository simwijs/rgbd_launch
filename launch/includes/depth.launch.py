from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manager'),
        DeclareLaunchArgument(
            'points_xyz',
            default_value="true"
        ),
        DeclareLaunchArgument(
            'rectify',
            default_value="true"
        ),
        DeclareLaunchArgument(
            'respawn',
            default_value="false"
        ),
        DeclareLaunchArgument('depth'),
        ComposableNodeContainer(
            name='component_container',
            namespace=LaunchConfiguration('manager'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    name=[LaunchConfiguration('depth'), '_rectify_depth'],
                    plugin='image_proc::RectifyNode',
                    remappings=[
                        ('image_mono', [LaunchConfiguration('depth'), '/image_raw']),
                        ('image_rect', [LaunchConfiguration('depth'), '/image_rect_raw'])
                    ],
                    parameters=[
                        {'interpolation': 0}
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                )
            ],
            condition=IfCondition(LaunchConfiguration('rectify'))
        ),
        ComposableNodeContainer(
            name='component_container',
            namespace=LaunchConfiguration('manager'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    name=[LaunchConfiguration('depth'), '_metric_rect'],
                    plugin='depth_image_proc::ConvertMetricNode',
                    remappings=[
                        ('image_raw', [LaunchConfiguration('depth'), '/image_rect_raw']),
                        ('image', [LaunchConfiguration('depth'), '/image_rect'])
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    name=[LaunchConfiguration('depth'), '_metric'],
                    plugin='depth_image_proc::ConvertMetricNode',
                    remappings=[
                        ('image_raw', [LaunchConfiguration('depth'), '/image_raw']),
                        ('image', [LaunchConfiguration('depth'), '/image'])
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]  
                )
            ]
        ),
        ComposableNodeContainer(
            name='component_container',
            namespace=LaunchConfiguration('manager'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    name=[LaunchConfiguration('depth'), '_points'],
                    plugin='depth_image_proc::PointCloudXyzNode',
                    remappings=[
                        ('image_rect', [LaunchConfiguration('depth'), '/image_rect_raw']),
                        ('points', [LaunchConfiguration('depth'), '/points'])
                    ],
                    parameters=[
                        {'interpolation': 0}
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]                
                )
            ],
            condition=IfCondition(LaunchConfiguration('points_xyz'))
        )
    ])
