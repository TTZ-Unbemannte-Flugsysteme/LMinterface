from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cocoa_dialogue'

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
    # Include templates folder for web interface
    package_data={
        'cocoa_dialogue': ['templates/*.html'],
    },
    include_package_data=True,
    install_requires=[
        'setuptools',
        'faster-whisper',
        'numpy',
        'flask',
    ],
    zip_safe=True,
    maintainer='ttz',
    maintainer_email='parikshistoryhithys7@gmail.com',
    description='Web interface for drone voice commands',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_interface_node = cocoa_dialogue.web_interface_node:main',
        ],
    },
)

