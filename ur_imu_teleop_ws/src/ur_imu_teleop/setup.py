from setuptools import setup

package_name = "ur_imu_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/imu_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="IMU-driven teleoperation helpers for UR5e MoveIt Servo.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_sim_node = ur_imu_teleop.imu_sim_node:main",
            "imu_servo_node = ur_imu_teleop.imu_servo_node:main",
        ],
    },
)
