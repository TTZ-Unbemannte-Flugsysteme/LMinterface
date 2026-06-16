from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cocoa_lga'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ttz',
    maintainer_email='parikshistoryhithys7@gmail.com',
    description='Language-Guided Abstraction for context filtering',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lga_node = cocoa_lga.lga_node:main',
        ],
    },
)
