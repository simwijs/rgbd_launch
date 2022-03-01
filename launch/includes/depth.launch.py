from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'points_xyz',
            default_value="true"
        ),
        DeclareLaunchArgument(
            'rectify',
            default_value="true"
        ),
        DeclareLaunchArgument(
            'camera_topic_prefix',
            default_value=""
            # default_value="camera/depth"
        ),
        DeclareLaunchArgument(
            'respawn',
            default_value="false"
        ),
        DeclareLaunchArgument('depth'),
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
                    package='image_proc',
                    name=[LaunchConfiguration('depth'), '_rectify_depth'],
                    plugin='image_proc::RectifyNode',
                    remappings=[
                        ('camera_info', [LaunchConfiguration('camera_topic_prefix'), '/camera_info']),
                        ('image', [LaunchConfiguration('camera_topic_prefix'), '/image_raw']),
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
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals('container', ''),
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    name=[LaunchConfiguration('depth'), '_metric_rect'],
                    plugin='depth_image_proc::ConvertMetricNode',
                    remappings=[
                        ('camera_info', [LaunchConfiguration('camera_topic_prefix'), '/camera_info']),
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
                        ('camera_info', [LaunchConfiguration('camera_topic_prefix'), '/camera_info']),
                        ('image_raw', [LaunchConfiguration('camera_topic_prefix'), '/image_raw']),
                        ('image', [LaunchConfiguration('camera_topic_prefix'), '/image'])
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]  
                )
            ]
        ),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    name=[LaunchConfiguration('depth'), '_points'],
                    plugin='depth_image_proc::PointCloudXyzNode',
                    remappings=[
                        ('camera_info', [LaunchConfiguration('camera_topic_prefix'), '/camera_info']),
                        ('image', [LaunchConfiguration('camera_topic_prefix'), '/image_raw']),
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
