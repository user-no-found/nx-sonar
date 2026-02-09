from setuptools import setup

package_name = "sonar_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nx-sonar",
    maintainer_email="dev@example.com",
    description="ROS2 sonar perception nodes.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_infer_node = sonar_perception.yolo_infer_node:main",
        ],
    },
)
