from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Motor controller loads bitstream & drives PWM
        Node(
            package='arlobot',
            executable='motor_controller.py',
            name='motor_controller',
            output='screen'
        ),
        # Encoder & odometry
        Node(package='arlobot', executable='encoder_logger_new.py', name='encoder_logger'),
        Node(package='arlobot', executable='odom_node.py', name='odom_node'),

        # Calibration test publisher
        Node(
            package='arlobot',
            executable='calibration_tester.py',
            name='calibration_tester',
            output='screen',
            parameters=[{
                'test_type': 'straight',
                'duration': 6.0,
                'linear_speed': 0.2,
                'angular_speed': 0.4,
                'log_file': '/tmp/calibration_log.csv'
            }]
        )
    ])
