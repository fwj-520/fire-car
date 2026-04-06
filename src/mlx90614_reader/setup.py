from setuptools import find_packages, setup

package_name = 'mlx90614_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'pyserial'],
    zip_safe=True,
    maintainer='gwm',
    maintainer_email='gwm@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mlx90614_reader = mlx90614_reader.mlx90614_reader:main',
        ],
    },
)
