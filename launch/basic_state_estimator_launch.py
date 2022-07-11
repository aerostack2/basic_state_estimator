from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        Node(
            package='basic_state_estimator',
            executable='basic_state_estimator_node',
            name='basic_state_estimator',
            namespace=LaunchConfiguration('drone_id'),
            output='screen',
            emulate_tty=True
        )
    ])