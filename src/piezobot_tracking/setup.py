#!/usr/bin/env python3
from setuptools import setup

package_name = "piezobot_tracking"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nils.Iseke@medma.uni-heidelberg.de",
    maintainer_email="Nils.Iseke@medma.uni-heidelberg.de",
    description="Contains the code to track the needle holder",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nnUnet = piezobot_tracking.nnUnet:main",
            "usb_reader = piezobot_tracking.usb_reader:main",
            "pose_from_segmentation = piezobot_tracking.pose_from_segmentation:main",
            "get_ik_solution = piezobot_tracking.get_ik_solution:main",
            "calculate_correction_pose = piezobot_tracking.calculate_correction_pose:main",
            "piezobot_joint_space_correction = piezobot_tracking.piezobot_joint_space_correction:main",
        ],
    },
)
