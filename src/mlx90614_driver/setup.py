from setuptools import setup

package_name = 'mlx90614_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mlx90614_serial.yaml']),
        ('share/' + package_name + '/launch', ['launch/mlx90614_serial.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='44577',
    maintainer_email='44577@qq.com',
    description='MLX90614 Infrared Temperature Sensor ROS 2 Driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mlx90614_node = mlx90614_driver.mlx90614_node:main',
        ],
    },
)
