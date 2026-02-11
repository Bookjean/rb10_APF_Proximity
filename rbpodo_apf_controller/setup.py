import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rbpodo_apf_controller'

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
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='song',
    maintainer_email='song@todo.todo',
    description='APF controller for RB10-1300E with ToF obstacle avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apf_controller = rbpodo_apf_controller.apf_node:main',
            'fake_tof_publisher = rbpodo_apf_controller.fake_tof_publisher:main',
            'go_home = rbpodo_apf_controller.go_home:main',
            'go_home_keep_obs = rbpodo_apf_controller.go_home_keep_obs:main',
        ],
    },
)
