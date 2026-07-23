from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('arlobot'), 'config')
    urdf_file = os.path.join(get_package_share_directory('arlobot'), 'urdf', 'arlo.urdf')

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_laser_broadcaster',
            arguments=['-0.125', '0', '0.135', '0', '0' ,'0', 'base_link' , 'laser'],
            output='screen'
        ),

        # Motor Controller
        Node(
            package='arlobot',
            executable='test_motor_controller',
            name='test_motor_controller',
            output='screen'
        ),

        # Encoder Logger
        Node(
            package='arlobot',
            executable='encoder_logger_new',
            name='encoder_logger_new',
            output='screen'
        ),

        # Odometry Node
        Node(
            package='arlobot',
            executable='odom_node_test',
            name='odom_node_test',
            output='screen'
            
        ),

        # RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }]
        ),
        

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
               os.path.join(config_dir, 'mapper_params_online_async.yaml'),
               ],
            remappings=[('scan', '/scan')]
        )
        #Node(
         #   package='tf2_ros',
          #  executable='static_transform_publisher',
           # name='static_map_to_odom',
            #arguments=['0', '0', '0', '0', '0' , '0', 'map' , 'odom'],
            #output='screen'
        #)

        
    ])
