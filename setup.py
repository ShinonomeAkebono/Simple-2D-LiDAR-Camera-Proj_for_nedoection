from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'lidar_camera_projection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'urdf'),glob('urdf/*.urdf')),
        (os.path.join('share',package_name,'launch'),glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='raitayamagishi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_camera_projection_node = lidar_camera_projection.lidar_camera_projection_node:main'
        ],
    },
)
