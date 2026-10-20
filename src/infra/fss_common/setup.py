from setuptools import find_packages, setup

package_name = "fss_common"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "hydra-core", "omegaconf"],
    zip_safe=True,
    maintainer="smong_jetson",
    maintainer_email="iyeons127@snu.ac.kr",
    description="FSS 공용 유틸리티",
    license="MIT",
    tests_require=["pytest"],
)
