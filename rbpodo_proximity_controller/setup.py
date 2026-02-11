import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rbpodo_proximity_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='song',
    maintainer_email='song@todo.todo',
    description='Proximity sensor-based APF controller with waypoint P2P motion for RB10-1300E',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proximity_controller = rbpodo_proximity_controller.proximity_controller_node:main',
            'waypoint_manager = rbpodo_proximity_controller.waypoint_manager:main',
            'fake_proximity_publisher = rbpodo_proximity_controller.fake_proximity_publisher:main',
        ],
    },
)
