from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('odom_only', default_value='True'),
        DeclareLaunchArgument('ground_truth', default_value='False'),
        DeclareLaunchArgument('sensor_fusion', default_value='False'),
        Node(
            package='basic_state_estimator',
            executable='basic_state_estimator_node',
            name='basic_state_estimator',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'odom_only': LaunchConfiguration('odom_only')},
                        {'ground_truth': LaunchConfiguration('ground_truth')},
                        {'sensor_fusion': LaunchConfiguration('sensor_fusion')}],
            output='screen',
            emulate_tty=True
        )
    ])
