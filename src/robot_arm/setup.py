from setuptools import find_packages, setup

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO Maintainer',
    maintainer_email='TODO@example.com',
    description='Starter structure for an autonomous robot arm.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
