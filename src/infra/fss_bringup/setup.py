import os
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "fss_bringup"
package_dir = Path(__file__).resolve().parent
config_root = package_dir.parents[2] / "config"
config_files = [
    (os.path.join("share", package_name, "config", str(directory.relative_to(config_root))),
     [os.path.relpath(path, package_dir) for path in sorted(directory.iterdir()) if path.is_file()])
    for directory in [config_root, *sorted(path for path in config_root.rglob("*") if path.is_dir())]
]

setup(
    name=package_name,
    version="0.2.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ] + config_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smong_jetson",
    maintainer_email="iyeons127@snu.ac.kr",
    description="FSS 런치 파일",
    license="MIT",
)
