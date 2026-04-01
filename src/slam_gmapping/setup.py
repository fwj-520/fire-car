from setuptools import setup
import os
from glob import glob

package_name = 'slam_gmapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         ['launch/slam_gmapping_launch.py']),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml') if glob('config/*.yaml') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@todo.com',
    description='Gmapping SLAM',
    license='TODO',
    tests_require=['pytest'],
    entry_points={},
)
