from glob import glob

from setuptools import setup

package_name = 'aircore_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # standalone tools: `ros2 run aircore_base measure_ticks.py` / `apply_measurements.py`
        ('lib/' + package_name, glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aircore',
    maintainer_email='aircore@example.com',
    description='Aircore base: STM32 serial bridge + differential drive motor controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'base_controller = aircore_base.base_controller:main',
            'odometry = aircore_base.odometry:main',
        ],
    },
)
