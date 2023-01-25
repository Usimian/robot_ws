import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = "my_planner_pkg"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.xml"))),
        (os.path.join(SHARE_DIR, "maps"), glob(os.path.join("maps", "*.yaml"))),
        (os.path.join(SHARE_DIR, "maps"), glob(os.path.join("maps", "*.pgm"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS 2 Developer",
    maintainer_email="jetson@todo.todo",
    description="Drive test",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
        ],
    },
)
