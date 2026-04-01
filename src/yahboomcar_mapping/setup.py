from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@todo.com',
    description='Simple mapping package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smooth_teleop = yahboomcar_mapping.smooth_teleop:main',
            'temperature_marker = yahboomcar_mapping.temperature_marker:main',
        ],
    },
)
