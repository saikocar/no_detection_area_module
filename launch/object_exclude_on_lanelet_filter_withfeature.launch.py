from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_ros_parameters(path, section):
    with open(path, 'r') as f:
        all_params = yaml.safe_load(f)
    section_data = all_params.get(section, {})
    return section_data.get('ros__parameters', {}) 

def generate_launch_description():
    config_file = os.path.join(
    get_package_share_directory('no_detection_area_module'),
    'config',
    'config.yaml'
)
    return LaunchDescription([
        Node(
            package='no_detection_area_module',
            executable='object_exclude_on_lanelet_filter_withfeature',
            name='object_exclude_on_lanelet_filter_withfeature',
            parameters=[
                os.path.join(
                    get_package_share_directory("no_detection_area_module"), "config", "config.yaml"
                )
            ],
            output='screen'
        )
    ])

