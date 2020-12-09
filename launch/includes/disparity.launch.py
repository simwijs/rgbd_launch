from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manager'),
        DeclareLaunchArgument(
            'depth',
            default_value='depth'
        ),
        DeclareLaunchArgument(
            'projector',
            default_value='projector'
        ),
        ComposableNodeContainer(
            name='component_container',
            namespace=LaunchConfiguration('manager'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    name=['disparity_', LaunchConfiguration('depth')],
                    plugin='depth_image_proc::DisparityNode',
                    remappings=[
                        ('left/image_rect', [LaunchConfiguration('depth'), '/image_rect_raw']),
                        ('right', [LaunchConfiguration('projector')]),
                        ('left/disparity', [LaunchConfiguration('depth'), '/disparity'])
                    ],
                    parameters=[
                        {'mini_range': 0.5},
                        {'max_range': 4.0}
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                )
            ]          
        )
    ])