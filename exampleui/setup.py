import os
from glob import glob

from setuptools import setup

package_name = 'example_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jens Ogorek',
    maintainer_email='jens@zauberzeug.com',
    description='Example UI using NiceGUI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_node = example_ui.ui_node:main',
        ],
    },
)
