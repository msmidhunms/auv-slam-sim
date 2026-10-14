from glob import glob

from setuptools import find_packages, setup

package_name = 'auv_sonar'

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
    description='Survey sonar models: multibeam swath, bathymetry submaps and sidescan waterfall',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multibeam_node = auv_sonar.multibeam_node:main',
            'swath_assembler_node = auv_sonar.swath_assembler_node:main',
            'sidescan_node = auv_sonar.sidescan_node:main',
        ],
    },
)
