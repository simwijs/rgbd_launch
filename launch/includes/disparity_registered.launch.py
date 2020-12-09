from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manager'),
        DeclareLaunchArgument(
            'depth_registered',
            default_value='depth_registered'
        ),
        DeclareLaunchArgument(
            'projector',
            default_value='projector'
        ),
        DeclareLaunchArgument(
            'sw_registered_processing',
            default_value="true"
        ),
        DeclareLaunchArgument(
            'hw_registered_processing',
            default_value="true"
        ),
        DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created.'
            )
        ),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    name='disparity_registered_sw',
                    plugin='depth_image_proc::DisparityNode',
                    remappings=[
                        ('left/image_rect', [LaunchConfiguration('depth_registered'), '/sw_registered/image_rect_raw']),
                        ('right', LaunchConfiguration('projector')),
                        ('left/disparity', [LaunchConfiguration('depth_registered'), '/disparity'])
                    ],
                    parameters=[
                        {'mini_range': 0.5},
                        {'max_range': 4.0}
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]                    
                )
            ],
            condition=IfCondition(LaunchConfiguration('sw_registered_processing'))            
        ),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    name='disparity_registered_hw',
                    plugin='depth_image_proc::DisparityNode',
                    remappings=[
                        ('left/image_rect', [LaunchConfiguration('depth_registered'), '/hw_registered/image_rect_raw']),
                        ('right', [LaunchConfiguration('projector')]),
                        ('left/disparity', [LaunchConfiguration('depth_registered'), '/disparity'])
                    ],
                    parameters=[
                        {'mini_range': 0.5},
                        {'max_range': 4.0}
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                )
            ],
            condition=IfCondition(LaunchConfiguration('sw_registered_processing'))
        )
    ])