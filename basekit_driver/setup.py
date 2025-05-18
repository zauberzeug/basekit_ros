
from setuptools import find_packages, setup

package_name = 'basekit_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'geometry_msgs', 'sensor_msgs'],
    zip_safe=True,
    maintainer='Zauberzeug GmbH',
    maintainer_email='ros@zauberzeug.com',
    description='This is a driver for running the a basekit robot.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basekit_driver_node = basekit_driver.basekit_driver_node:main'
        ],
    },
)
