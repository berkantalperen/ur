from setuptools import setup
import os
from glob import glob

package_name = 'imu_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- ADD THESE TWO LINES ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'scipy', 'matplotlib', 'PyQt5'],
    zip_safe=True,
    maintainer='berkant',
    maintainer_email='berkant@todo.todo',
    description='Teleoperation IMU Simulator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_sim = imu_sim.imu_sim_server:main',
            'safety_node = imu_sim.safety_node:main',
            'clock_bridge = imu_sim.clock_bridge:main',
        ],
    },
)