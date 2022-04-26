import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

camera_calibration_pkg_prefix = get_package_share_directory('camera_calibration')
camera_calibration_param_file = os.path.join(camera_calibration_pkg_prefix, 'param/params.yaml')


def generate_launch_description():
    camera_calibration_node = Node(
        package='camera_calibration',
        executable='camera_calibration_node_exe',
        namespace='camera_calibration',
        parameters=[camera_calibration_param_file]
    )

    return launch.LaunchDescription([camera_calibration_node])
