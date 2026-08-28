import os
from glob import glob

from setuptools import setup

package_name = "lbr_dual_arm_pdz_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PDZ",
    maintainer_email="pdz@todo.todo",
    description="Gazebo bringup + plan orchestrator for the dual-arm KUKA + pdz gripper plumbers_block replay.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plan_executor_node = lbr_dual_arm_pdz_bringup.plan_executor_node:main",
            "hardware_plan_executor_node = lbr_dual_arm_pdz_bringup.hardware_plan_executor_node:main",
            "variable_impedance_plan_executor_node = lbr_dual_arm_pdz_bringup.variable_impedance_plan_executor_node:main",
        ],
    },
)
