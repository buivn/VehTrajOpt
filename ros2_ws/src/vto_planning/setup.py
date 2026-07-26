from setuptools import find_packages, setup

package_name = "vto_planning"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hoang-Dung Bui",
    maintainer_email="bui.hoangdungtn@gmail.com",
    description="Global planners: A* (grid search) now; Hybrid A* / C++ bridge later.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "astar_planner = vto_planning.astar_planner_node:main",
        ],
    },
)
