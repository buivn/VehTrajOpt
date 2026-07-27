import glob

from setuptools import find_packages, setup

package_name = "vto_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hoang-Dung Bui",
    maintainer_email="bui.hoangdungtn@gmail.com",
    description="Path-tracking controllers: Pure Pursuit, MPC, MPPI.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "pure_pursuit = vto_control.pure_pursuit_node:main",
            "mpc_controller = vto_control.mpc_controller_node:main",
            "mppi_controller = vto_control.mppi_controller_node:main",
            "path_publisher = vto_control.path_publisher_node:main",
        ],
    },
)
