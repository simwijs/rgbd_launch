from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manager'),
        DeclareLaunchArgument(
            'respawn',
            default_value="false"
        ),
        DeclareLaunchArgument('rgb'),
        DeclareLaunchArgument(
            'debayer_processing',
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
            condition=IfCondition(LaunchConfiguration('debayer_processing')),
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    name=[LaunchConfiguration('rgb'), '_debayer'],
                    plugin='image_proc::DebayerNode',
                    remappings=[
                        ('image_raw', [LaunchConfiguration('rgb'), '/image_raw']),
                        ('image_mono', [LaunchConfiguration('rgb'), '/image_mono']),
                        ('image_color', [LaunchConfiguration('rgb'), '/image_color'])
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                ),
                ComposableNode(
                    package='image_proc',
                    name=[LaunchConfiguration('rgb'), '_rectify_mono'],
                    plugin='image_proc::RectifyNode',
                    remappings=[
                        ('image_mono', [LaunchConfiguration('rgb'), '/image_mono']),
                        ('image_rect', [LaunchConfiguration('rgb'), '/image_rect_mono']),
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                ),
                ComposableNode(
                    package='image_proc',
                    name=[LaunchConfiguration('rgb'), '_rectify_color'],
                    plugin='image_proc::RectifyNode',
                    remappings=[
                        ('image_mono', [LaunchConfiguration('rgb'), '/image_color']),
                        ('image_rect', [LaunchConfiguration('rgb'), '/image_rect_color']),
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                )
            ],
        ),
        LoadComposableNodes(
            condition=UnlessCondition(LaunchConfiguration('debayer_processing')),
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    name=[LaunchConfiguration('rgb'), '_rectify_color'],
                    plugin='image_proc::RectifyNode',
                    remappings=[
                        ('image_mono', [LaunchConfiguration('rgb'), '/image_raw']),
                        ('image_rect', [LaunchConfiguration('rgb'), '/image_rect_color']),
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                )                
            ],
        )
    ])