"""fss_arm ROS2 패키지 설치 정의."""

from setuptools import find_packages, setup

setup(
    name="fss_arm",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fss_arm"]),
        ("share/fss_arm", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="smong_jetson",
    maintainer_email="iyeons127@snu.ac.kr",
    description="FSS 팔 위치·속도·전류 명령 및 Dynamixel 연결",
    license="MIT",
    entry_points={"console_scripts": ["arm_driver_node = fss_arm.arm_driver_node:main"]},
)
