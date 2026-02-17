from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur_servo_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkantalperen',
    maintainer_email='berkantalperen@gmail.com',
    description='UR MoveIt Servo keyboard demo + owned servo config',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_keyboard = ur_servo_demo.servo_keyboard:main',
            'joint_slider = ur_servo_demo.joint_slider:main',  # <--- ADD THIS
            'pose_demo = ur_servo_demo.pose_demo:main', # <--- Add this
        ],
    },
)

