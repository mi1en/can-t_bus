import os
from glob import glob
from setuptools import find_packages, setup

package_name = "vesc_can_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="plesh",
    maintainer_email="pleshevichmilena186@gmail.com",
    description="VESC motor control over CAN via Waveshare USB-CAN-A (ROS 2).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vesc_can_node = vesc_can_driver.vesc_can_node:main",
            "vesc_teleop_node = vesc_can_driver.vesc_teleop_node:main",
        ],
    },
)
