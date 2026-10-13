from setuptools import find_packages, setup

package_name = 'auv_teleop'

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
    maintainer='Midhun M S',
    maintainer_email='msmidhunms@gmail.com',
    description='Keyboard teleop for the vehicle, publishes cmd_vel continuously',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_node = auv_teleop.keyboard_node:main',
        ],
    },
)
