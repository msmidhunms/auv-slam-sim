from glob import glob

from setuptools import find_packages, setup

package_name = 'auv_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Midhun M S',
    maintainer_email='msmidhunms@gmail.com',
    description='Live comparison of the EKF and SLAM estimates against the simulator ground truth',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eval_node = auv_eval.eval_node:main',
        ],
    },
)
