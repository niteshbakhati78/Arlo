import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # For robot_state_publisher
    map_yaml = os.path.join(get_package_share_directory('arlobot'), 'maps', 'room1_test2.yaml')
    nav2_params= os.path.join(get_package_share_directory('arlobot'), 'config', 'nav2_params_new.yaml')
    config_dir = os.path.join(get_package_share_directory('arlobot'), 'config')
    urdf = os.path.join(get_package_share_directory('arlobot'), 'urdf', 'arlobot.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
        
    #Helper to create a v4l2 camera node with namespace + unique frame_id
    def cam_node(ns: str, device: str, frame_id: str):
    	return Node(
    	    package='v4l2_camera',
    	    executable='v4l2_camera_node',
            name='camera',
            namespace=ns,
            output='screen',
            parameters=[{
                'video_device': device,
                'image_size': [320, 240],
                'frame_rate': 10.0,
                'pixel_format': 'YUYV',
                'output_encoding': 'yuv422_yuy2',
                'frame_id': frame_id
            }]
            
    	)       


    return LaunchDescription([
     
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'publish_frequency': 5.0
                },
                {
                    'robot_description': robot_description,
                }
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_to_laser_broadcaster',
            arguments=['-0.125', '0', '0.165', '0', '0' ,'0', 'base_link' , 'laser'],
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
        #3x USB WebCams
       cam_node('cam1', '/dev/video0', 'cam1_camera_link'),
       cam_node('cam2', '/dev/video2', 'cam2_camera_link'),
       #cam_node('cam3', '/dev/video4', 'cam3_camera_link'),
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
        ),
        

        # Navigation via NAV2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'nav2_bringup'), 'launch/navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'use_respawn': 'true',
                
                'params_file' : os.path.join(get_package_share_directory('arlobot'), 'config', 'nav2_params.yaml'),
                'enable_map_server': 'false', 
                'enable_amcl': 'false',
                #'map' : os.path.join(get_package_share_directory('arlobot'), 'maps', 'room1_test2.yaml')
            }.items()
        ),

        # SLAM via slam_toolbox
        #IncludeLaunchDescription(
         #   PythonLaunchDescriptionSource([
          #      PathJoinSubstitution([
           #         FindPackageShare('slam_toolbox'),
            #        'launch',
             #       'online_async_launch.py'
              #  ])
           # ]),
            #launch_arguments={
             #   'slam_params_file': TextSubstitution(text=str(PathJoinSubstitution([
              #      FindPackageShare('arlobot'),
               #     'config',
                #    'mapper_params_online_async.yaml'
                #])))
           # }.items()
        #),
         #TimerAction(
          #  period=5.0,  # delay start by 5 seconds
          #  actions=[
                 #Camera Publisher (OpenCV + GStreamer)
           #     Node(
            #        package='arlobot',
             #       executable='camera_pub',
              #      name='camera_pub',
               #     output='screen',
                #),

                 #Static TF: base_link ? camera_link
                #Node(
                 #   package='tf2_ros',
                  #  executable='static_transform_publisher',
                   # name='base_to_camera_broadcaster',
                    #arguments=['0.202', '0.0', '0.285', '0', '0', '0', 'base_link', 'camera_link'],
                    #output='screen'
               # ),
           # ]
        #),
    ])
    



if __name__ == '__main__':
    generate_launch_description()
