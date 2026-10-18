import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description', 'urdf'), glob('description/urdf/*.urdf')),
        (os.path.join('share', package_name, 'description', 'meshes'), glob('description/meshes/*.stl')),
        (os.path.join('share', package_name, 'description', 'rviz'), glob('description/rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='TODO Maintainer',
    maintainer_email='TODO@example.com',
    description='Autonomous robot arm: description, kinematics, calibration, autonomy.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
