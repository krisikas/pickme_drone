from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pickme_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@pickmedrone.local',
    description='PX4 flight command interface for PickmeDrone',
    license='MIT',
    entry_points={
        'console_scripts': [
            'commander_node = pickme_core.commander_node:main',
        ],
    },
)
