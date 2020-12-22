from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import LoadComposableNodes
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'respawn',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'sw_registered_processing',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'hw_registered_processing',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'rgb',
            default_value='rgb'
        ),
        DeclareLaunchArgument(
            'depth',
            default_value='depth'
        ),
        DeclareLaunchArgument(
            'depth_registered',
            default_value='depth_registered'
        ),
        DeclareLaunchArgument(
            'suffix',
            default_value=[LaunchConfiguration('depth'), '_', LaunchConfiguration('rgb')]
        ),
        DeclareLaunchArgument(
            'depth_registered_filtered',
            default_value='depth_registered'
        ),
        DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created.'
            )
        ),
        GroupAction(
            actions=[
                LoadComposableNodes(
                    condition=LaunchConfigurationNotEquals('container', ''),
                    target_container=LaunchConfiguration('container'), 
                    composable_node_descriptions=[
                        ComposableNode(
                            package='depth_image_proc',
                            plugin='depth_image_proc::RegisterNode',
                            name=['register_', LaunchConfiguration('suffix')],
                            remappings=[
                                ('rgb/camera_info', [LaunchConfiguration('rgb'), '/camera_info']),
                                ('depth/camera_info', [LaunchConfiguration('depth'), '/camera_info']),
                                ('depth/image_rect', [LaunchConfiguration('depth'), '/image_rect_raw']),
                                ('depth_registered/image_rect', [LaunchConfiguration('depth_registered'), '/sw_registered/image_rect_raw'])
                            ],
                            extra_arguments=[
                                {'--no-daemon': LaunchConfiguration('respawn')}
                            ]
                        ),
                        ComposableNode(
                            package='depth_image_proc',
                            name='points_xyzrgb_sw_registered',
                            plugin='depth_image_proc::PointCloudXyzrgbNode',
                            remappings=[
                                ('rgb/image_rect_color', [LaunchConfiguration('rgb'), '/image_rect_color']),
                                ('rgb/camera_info', [LaunchConfiguration('rgb'), '/camera_info']),
                                ('depth_registered/image_rect', [LaunchConfiguration('depth_registered_filtered'), '/sw_registered/image_rect_raw']),
                                ('depth_registered/points', [LaunchConfiguration('depth_registered'), '/points'])
                            ],
                            extra_arguments=[
                                {'--no-daemon': LaunchConfiguration('respawn')}
                            ]                       
                        ),
                        ComposableNode(
                            package='depth_image_proc',
                            name=[LaunchConfiguration('depth_registered'), '_sw_metric_rect'],
                            plugin='depth_image_proc::ConvertMetricNode',
                            remappings=[
                                ('image_raw', [LaunchConfiguration('depth_registered'), '/sw_registered/image_rect_raw']),
                                ('image', [LaunchConfiguration('depth_registered'), '/sw_registered/image_rect'])
                            ],                           
                            extra_arguments=[
                                {'--no-daemon': LaunchConfiguration('respawn')}
                            ]                       
                        )
                    ]
                )
            ],
            condition=IfCondition(LaunchConfiguration('sw_registered_processing'))
        ),
        GroupAction(
            actions=[
                LoadComposableNodes(
                    condition=LaunchConfigurationNotEquals('container', ''),
                    target_container=LaunchConfiguration('container'), 
                    composable_node_descriptions=[
                        ComposableNode(
                            package='image_proc',
                            name=[LaunchConfiguration('depth_registered'), '_rectify_depth'],
                            plugin='image_proc::RectifyNode',
                            remappings=[
                                ('image_mono', [LaunchConfiguration('depth_registered'), '/image_raw']),
                                ('image_rect', [LaunchConfiguration('depth_registered'), '/hw_registered/image_rect_raw'])
                            ],
                            parameters=[
                                {'interpolation': 0}
                            ],                            
                            extra_arguments=[
                                {'--no-daemon': LaunchConfiguration('respawn')}
                            ]                       
                        ),
                        ComposableNode(
                            package='depth_image_proc',
                            name='points_xyzrgb_hw_registered',
                            plugin='depth_image_proc::PointCloudXyzrgbNode',
                            remappings=[
                                ('rgb/image_rect_color', [LaunchConfiguration('rgb'), '/image_rect_color']),
                                ('rgb/camera_info', [LaunchConfiguration('rgb'), '/camera_info']),
                                ('depth_registered/image_rect', [LaunchConfiguration('depth_registered_filtered'), '/hw_registered/image_rect_raw']),
                                ('depth_registered/points', [LaunchConfiguration('depth_registered'), '/points'])
                            ],                         
                            extra_arguments=[
                                {'--no-daemon': LaunchConfiguration('respawn')}
                            ]                       
                        ),
                        ComposableNode(
                            package='depth_image_proc',
                            name=[LaunchConfiguration('depth_registered'), '_hw_metric_rect'],
                            plugin='depth_image_proc::ConvertMetricNode',
                            remappings=[
                                ('image_raw', [LaunchConfiguration('depth_registered'), '/hw_registered/image_rect_raw']),
                                ('image', [LaunchConfiguration('depth_registered'), '/hw_registered/image_rect'])
                            ],                            
                            extra_arguments=[
                                {'--no-daemon': LaunchConfiguration('respawn')}
                            ]                       
                        ),
                        ComposableNode(
                            package='depth_image_proc',
                            name=[LaunchConfiguration('depth_registered'), '_metric'],
                            plugin='depth_image_proc::ConvertMetricNode',
                            remappings=[
                                ('image_raw', [LaunchConfiguration('depth_registered'), '/image_raw']),
                                ('image', [LaunchConfiguration('depth_registered'), '/image'])
                            ],                           
                            extra_arguments=[
                                {'--no-daemon': LaunchConfiguration('respawn')}
                            ]                       
                        )
                    ]
                )
            ],
            condition=IfCondition(LaunchConfiguration('hw_registered_processing'))
        )
    ])