from setuptools import setup
import os
from glob import glob

package_name = 'ur_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include Config Files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkantalperen',
    maintainer_email='berkantalperen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_controller = ur_teleop.pose_controller:main',
            'hand_pose_sim = ur_teleop.hand_pose_sim:main',  # <--- ADD THIS
            'imu_real_driver = ur_teleop.imu_real_driver:main', # <--- ADD THIS
        ],
    },
)