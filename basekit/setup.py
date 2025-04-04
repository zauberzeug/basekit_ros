import os
from glob import glob

from setuptools import setup

package_name = 'basekit'

setup(
    name=package_name,
    version='0.1.0',
    description='todo',
    author='Zauberzeug GmbH',
    author_email='ros@zauberzeug.com',
    python_requires='>=3.11, <3.13',
    packages=['basekit'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('lib', package_name), []),  # Create the lib/basekit directory
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Zauberzeug GmbH',
    maintainer_email='ros@zauberzeug.com',
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    license='MIT',
    entry_points={
        'console_scripts': [
            'basekit_ui = basekit.ui:main',
            'basekit_simulator = basekit.simulator:main',
            'basekit_camera = basekit.camera:main',
        ],
    },
)
