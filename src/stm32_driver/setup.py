from setuptools import setup
import os
from glob import glob

package_name = 'stm32_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lu ban mao',
    maintainer_email='user@todo.com',
    description='STM32 motor driver',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hiwonder_motor_driver = stm32_driver.stm32_motor_driver:main',
        ],
    },
)
