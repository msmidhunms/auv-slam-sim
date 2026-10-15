from glob import glob

from setuptools import find_packages, setup

package_name = 'auv_mission'

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
    description='Survey mission state machine with line following, surfacing for GPS and abort rules',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = auv_mission.mission_node:main',
        ],
    },
)
