from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 这里需要 yahboomcar_nav 目录存在
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@todo.com',
    description='Navigation package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={},
)
