from glob import glob

from setuptools import find_packages, setup

package_name = 'auv_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Midhun M S',
    maintainer_email='msmidhunms@gmail.com',
    description='Simulated navigation sensors (AHRS, DVL, depth, GPS, USBL) with realistic noise models',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ahrs_node = auv_sensors.ahrs_node:main',
            'dvl_node = auv_sensors.dvl_node:main',
            'depth_node = auv_sensors.depth_node:main',
            'gps_node = auv_sensors.gps_node:main',
            'usbl_node = auv_sensors.usbl_node:main',
        ],
    },
)
