from glob import glob

from setuptools import setup

package_name = 'aircore_sim'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        (share + '/launch', glob('launch/*.launch.py')),
        (share + '/models/farm_map', ['models/farm_map/model.sdf', 'models/farm_map/model.config']),
        (share + '/models/farm_map/meshes', glob('models/farm_map/meshes/*.stl')),
        (share + '/worlds', glob('worlds/*.sdf')),
        (share + '/maps', glob('maps/*.pgm') + glob('maps/*.yaml')),
        ('lib/' + package_name, ['scripts/generate_world.py', 'scripts/make_occupancy_map.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aircore',
    maintainer_email='aircore@example.com',
    description='Aircore smart-farm simulation: Gazebo Fortress world from STL map + Nav2 2D map',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
