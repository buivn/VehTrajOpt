import glob

from setuptools import find_packages, setup

package_name = "vto_bench"

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
    description="Controller/planner benchmark: goal-tour harness + metrics for the report.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "benchmark = vto_bench.benchmark_node:main",
        ],
    },
)
