import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'field_friend_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='tschuette@atb-potsdam.de, dkloeser@atb-potsdam.de',
    description='This is a driver for running the field friend hardware.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'field_friend_driver_node = field_friend_driver.field_friend_driver_node:main'
        ],
    },
)
