from setuptools import setup, find_packages

package_name = 'cocoa_path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker file
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Config files
        ('share/' + package_name + '/config', ['config/planner_params.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='A* Path Planning Service for Cocoa Drone System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This creates the 'ros2 run cocoa_path_planner path_planner_node' command
            'path_planner_node = cocoa_path_planner.path_planner_node:main',
        ],
    },
)
