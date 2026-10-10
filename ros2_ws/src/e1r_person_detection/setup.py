from glob import glob

from setuptools import setup

package_name = "e1r_person_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="simhatel",
    maintainer_email="simhatelrobotics@gmail.com",
    description="Visualisation and geometric person detection for the RoboSense E1R lidar",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_detector = e1r_person_detection.person_detector_node:main",
            "fake_lidar = e1r_person_detection.fake_lidar_node:main",
        ],
    },
)
