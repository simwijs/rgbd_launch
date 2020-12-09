import launch
import launch_ros.actions
import launch_testing.actions
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rgbd_launch_dir = get_package_share_directory('rgbd_launch')
    return LaunchDescription([
        DeclareLaunchArgument(
            'manager',
            default_value='camera_nodelet_manager'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/includes/processing.launch.py'),
            launch_arguments=[
                ('manager', LaunchConfiguration('manager')),
                ('rgb_processing', 'true'),
                ('ir_processing', 'true'),
                ('depth_processing', 'true'),
                ('depth_registered_processing', 'true'),
                ('disparity_processing', 'true'),
                ('disparity_registered_processing', 'true'),
                ('hw_registered_processing', 'true'),
                ('sw_registered_processing', 'true')
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/launch/foxy_frames.launch.py')
        )
    ])