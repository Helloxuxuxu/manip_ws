from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rb210_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
  	('share/' + package_name+'/urdf/', glob('urdf/*')),
  	('share/' + package_name+'/rviz/', glob('rviz/*')),
    ('share/' + package_name+'/worlds/', glob('worlds/*')),
  	('share/' + package_name+'/meshes/', glob('meshes/*')),
    ('share/' + package_name+'/collision/', glob('collision/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='olmer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
