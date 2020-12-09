from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription([
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
            'remap',
            default_value=[LaunchConfiguration('rgb'), '/image_color'],
            condition=IfCondition(LaunchConfiguration('debayer_processing')),
        ),
        DeclareLaunchArgument(
            'remap',
            default_value=[LaunchConfiguration('rgb'), '/image_raw'],
            condition=UnlessCondition(LaunchConfiguration('debayer_processing')), 
        )
    ])
    container = ComposableNodeContainer(
        name='component_container',
        namespace=LaunchConfiguration('manager'),
        package='rclcpp_components',
        executable='component_container',
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
            )
        ],
        condition=IfCondition(LaunchConfiguration('debayer_processing'))
    )
    loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                name=[LaunchConfiguration('rgb'), '_rectify_color'],
                plugin='image_proc::RectifyNode',
                remappings=[
                    ('image_mono', [LaunchConfiguration('remap')]),
                    ('image_rect', [LaunchConfiguration('rgb'), '/image_rect_color']),
                ],
                extra_arguments=[
                    {'--no-daemon': LaunchConfiguration('respawn')}
                ]
            )
        ],
        target_container=container
    )
    ld.add_action(container)
    ld.add_action(loader)
    return ld