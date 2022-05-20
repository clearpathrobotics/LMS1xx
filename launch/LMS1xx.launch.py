import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ROS packages
    sick_lms1xx_pkg = get_package_share_directory('LMS1xx')

    # config
    sensor_config = os.path.join(sick_lms1xx_pkg, 'config', 'lms_111.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Nodes
    lms1xx_driver = Node(
        package='lms1xx',
        executable='lms1xx',
        name='lms1xx',
        parameters=[sensor_config],
        output='screen',
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time if true'),
        # Nodes
        lms1xx_driver,
    ])