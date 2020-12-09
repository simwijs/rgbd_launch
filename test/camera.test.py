import launch
import launch_ros.actions
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_test_description():
    rgbd_launch_dir = get_package_share_directory('rgbd_launch')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rgbd_launch_dir + '/test/camera_for_test.launch.py')
        ),
        launch.actions.ExecuteProcess(
            name='test_foxy_frames',
            cmd=rgbd_launch_dir + '/test/test_rgbd_launch.py'
        ),
        launch_testing.actions.ReadyToTest()
    ])