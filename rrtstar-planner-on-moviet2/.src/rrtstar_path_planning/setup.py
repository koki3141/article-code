from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'rrtstar_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_dir={"": "."},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='shimohara7192@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"{package_name} = {package_name}.{package_name}:main",
            f"sample_robot_state_client_node = sample.sample_robot_state_client_node:main",
        ],
    },
)
