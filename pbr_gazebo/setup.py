import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pbr_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'Media', 'models'), glob('Media/models/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marc Vinci',
    maintainer_email='marc.vinci@dfki.de',
    description='This package contains files for pbr simulation with gazebo.',
    license='BSD-3-Clause',
    scripts=[
            'nodes/gazebo_unpauser',
            'nodes/gazebo_object_pose_helper.py',
        ]
)
