import glob

from setuptools import find_packages, setup


package_name = "franka_vr_adapter"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jianshu Hu",
    maintainer_email="jianshuhu123@gmail.com",
    description="Safety-gated Quest 3 joint target adapter for the right FR3 arm.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "adapter_node = franka_vr_adapter.adapter_node:main",
            "closed_loop_runner = franka_vr_adapter.closed_loop_runner:main",
            "fault_injection_runner = franka_vr_adapter.fault_injection_runner:main",
        ],
    },
)
