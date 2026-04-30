from setuptools import setup, find_packages

package_name = 'dt_teleop_testbed'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/waypoints_example.yaml',
            'config/replay_example.csv',
            'config/network_profiles.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/testbed.launch.py',
        ]),
        ('share/' + package_name + '/scripts', [
            'scripts/export_lite6_urdf_for_unity.sh',
            'scripts/lite6_enable.sh',
            'scripts/preflight.py',
            'scripts/plot_experiment.py',
            'scripts/compute_metrics.py',
            'scripts/plot_metrics.py',
            'scripts/aggregate_runs.py',
        ]),
        ('share/' + package_name + '/unity_scripts', [
            'unity_scripts/TwinJointStatePublisher.cs',
            'unity_scripts/JointSliderController.cs',
            'unity_scripts/RealRobotStateSubscriber.cs',
            'unity_scripts/GripperController.cs',
            'unity_scripts/BrickPoseSubscriber.cs',
        ]),
        ('share/' + package_name + '/unity_scripts/Editor', [
            'unity_scripts/Editor/ArticulationDriveSetup.cs',
        ]),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='DT teleoperation testbed for UFactory Lite 6',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'headless_twin = dt_teleop_testbed.headless_twin_node:main',
            'mock_lite6 = dt_teleop_testbed.mock_lite6_node:main',
            'aoi_logger = dt_teleop_testbed.aoi_logger_node:main',
            'network_conditioner = dt_teleop_testbed.network_conditioner_node:main',
            'twin_to_lite6_bridge = dt_teleop_testbed.twin_to_lite6_bridge_node:main',
            'experiment_runner = dt_teleop_testbed.experiment_runner_node:main',
            'gripper_bridge = dt_teleop_testbed.gripper_bridge_node:main',
            'scripted_operator = dt_teleop_testbed.scripted_operator_node:main',
            'aruco_detector = dt_teleop_testbed.aruco_detector_node:main',
        ],
    },
)
