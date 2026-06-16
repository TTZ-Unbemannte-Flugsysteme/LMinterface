from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cocoa_swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ttz',
    maintainer_email='parikshistoryhithys7@gmail.com',
    description='Swarm coordination layer for COCOA',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_manager_node = cocoa_swarm.swarm_manager_node:main',
        ],
    },
)
