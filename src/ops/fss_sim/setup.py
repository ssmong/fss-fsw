from setuptools import setup

package_name = "fss_sim"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smong",
    maintainer_email="iyeons127@snu.ac.kr",
    description="FSS SIL 플랜트 모델",
    license="MIT",
    entry_points={
        "console_scripts": [
            "plant_model_node = fss_sim.plant_model_node:main",
        ],
    },
)
