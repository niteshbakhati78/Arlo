from setuptools import find_packages, setup

package_name = 'arlobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch' , [
            'launch/arlobot_navigation_launch.py',
            'launch/arlobot_slam_launch.py',
            'launch/navigation_launch.py',
            'launch/localization_launch.py',
            'launch/calibration_test.launch.py'
        ]),
        ('share/' + package_name + '/urdf', ['urdf/arlobot.urdf']),
        ('share/' + package_name + '/config', [
        
            'config/costmap_common_params.yaml',
            'config/global_costmap_params.yaml',
            'config/local_costmap_params.yaml',
            'config/move_base_params.yaml',
            'config/slam_toolbox_params.yaml',
            'config/mapper_params_online_async.yaml',
            'config/nav2_params.yaml',
            'config/nav2_params_new.yaml'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_logger = arlobot.encoder_logger:main',
            'odom_node = arlobot.odom_node:main',
            'motor_controller = arlobot.motor_controller:main',
            'encoder_logger_new = arlobot.encoder_logger_new:main',
            'odom_node_test = arlobot.odom_node_test:main',
            'odom_node_new = arlobot.odom_node_new:main',
            'motor_controller_new = arlobot.motor_controller_new:main',
            'test_motor_controller = arlobot.test_motor_controller:main',
            'camera_pub = arlobot.camera_pub:main',
            'calibration_tester = arlobot.calibration_tester:main'
        ],
    },
)
