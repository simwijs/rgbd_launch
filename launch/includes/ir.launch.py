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
            'respawn',
            default_value='false'
        ),
        DeclareLaunchArgument('ir'),
        ComposableNodeContainer(
            name='component_container',
            namespace=LaunchConfiguration('manager'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    name=[LaunchConfiguration('ir'), '_rectify_ir'],
                    plugin='image_proc::RectifyNode',
                    remappings=[
                        ('image_mono', [LaunchConfiguration('ir'), '/image_raw']),
                        ('image_rect', [LaunchConfiguration('ir'), '/image_rect_ir'])
                    ],
                    extra_arguments=[
                        {'--no-daemon': LaunchConfiguration('respawn')}
                    ]
                )
            ]            
        )
    ])