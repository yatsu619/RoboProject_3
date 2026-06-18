from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.action import IncludeLaunchDescription
import os 
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
        
    serial_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ro45_ros2_pickrobot_serial'),
                'launch',
                'launch_nodes.py'
                )
            )
        )

    return LaunchDescription([
        serial_launch, 
        Node(
            package='pickme_dev',
            executable='motioncontroller_node'
        ),
        Node(
            package='pickme_dev',
            executable='WaypointPredition_node'
        ),
        Node(
            package='pickme_dev',
            executable='delay_buffer_node'
        ),
        Node(
            package='pickme_dev',
            executable='cam_node '
        ),
    ])