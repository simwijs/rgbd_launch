from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.actions import SetLaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def arg(name, value):
    return DeclareLaunchArgument(
        name,
        default_value=value
    )

def generate_launch_description():
    rgbd_launch_dir = get_package_share_directory('rgbd_launch')
    return LaunchDescription([
        arg('respawn', 'false'),
        arg('rgb_processing', "true"),
        arg('debayer_processing', "true"),
        arg('ir_processing', "true"),
        arg('depth_processing', "true"),
        arg('depth_registered_processing', "true"),
        arg('disparity_processing', "true"),
        arg('disparity_registered_processing', "true"),
        arg('sw_registered_processing', "true"),
        arg('hw_registered_processing', "true"),
        arg('rgb', 'rgb'),
        arg('ir', 'ir'),
        arg('depth', 'depth'),
        arg('depth_registered', 'depth_registered'),
        arg('depth_registered_filtered', 'depth_registered'),
        arg('projector', 'projector'),
        DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created.'
            )
        ),
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', ''),
            package='rclcpp_components',
            executable='component_container',
            name='component_container',
            namespace='',
        ),
        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals('container', ''),
            name='container',
            value='component_container'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/rgb.launch.py'),
            launch_arguments=[
                ('container', LaunchConfiguration('container')),
                ('respawn', LaunchConfiguration('respawn')),
                ('rgb', LaunchConfiguration('rgb')),
                ('debayer_processing', LaunchConfiguration('debayer_processing'))
            ],
            condition=IfCondition(LaunchConfiguration('rgb_processing'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/ir.launch.py'),
            launch_arguments=[
                ('container', LaunchConfiguration('container')),
                ('respawn', LaunchConfiguration('respawn')),
                ('ir', LaunchConfiguration('ir'))
            ],
            condition=IfCondition(LaunchConfiguration('ir_processing'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/depth.launch.py'),
            launch_arguments=[
                ('container', LaunchConfiguration('container')),
                ('respawn', LaunchConfiguration('respawn')),
                ('depth', LaunchConfiguration('depth'))
            ],
            condition=IfCondition(LaunchConfiguration('depth_processing'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/depth_registered.launch.py'),
            launch_arguments=[
                ('container', LaunchConfiguration('container')),
                ('rgb', LaunchConfiguration('rgb')),
                ('depth', LaunchConfiguration('depth')),
                ('depth_registered', LaunchConfiguration('depth_registered')),
                ('depth_registered_filtered', LaunchConfiguration('depth_registered_filtered')),
                ('respawn', LaunchConfiguration('respawn')),
                ('sw_registered_processing', LaunchConfiguration('sw_registered_processing')),
                ('hw_registered_processing', LaunchConfiguration('hw_registered_processing'))
            ],
            condition=IfCondition(LaunchConfiguration('depth_registered_processing'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/disparity.launch.py'),
            launch_arguments=[
                ('container', LaunchConfiguration('container')),
                ('depth', LaunchConfiguration('depth')),
                ('projector', LaunchConfiguration('projector')),
                ('respawn', LaunchConfiguration('respawn'))
            ],
            condition=IfCondition(LaunchConfiguration('disparity_processing'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/disparity_registered.launch.py'),
            launch_arguments=[
                ('container', LaunchConfiguration('container')),
                ('depth_registered', LaunchConfiguration('depth_registered')),
                ('projector', LaunchConfiguration('projector')),
                ('respawn', LaunchConfiguration('respawn')),
                ('rgb', LaunchConfiguration('rgb')),
                ('sw_registered_processing', LaunchConfiguration('sw_registered_processing')),
                ('hw_registered_processing', LaunchConfiguration('hw_registered_processing'))
            ],
            condition=IfCondition(LaunchConfiguration('disparity_registered_processing'))
        )
    ])