from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lu ban mao',
    maintainer_email='user@todo.com',
    description='Laser avoidance',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_avoidance = yahboomcar_avoidance.simple_avoidance:main',
            'avoidance_mapping = yahboomcar_avoidance.avoidance_mapping:main',
            'simple_odom = yahboomcar_avoidance.simple_odom:main',
            'slam_avoidance = yahboomcar_avoidance.slam_avoidance:main',
            'teleop_avoidance_stop = yahboomcar_avoidance.teleop_avoidance_stop:main',
        ],
    },
)
