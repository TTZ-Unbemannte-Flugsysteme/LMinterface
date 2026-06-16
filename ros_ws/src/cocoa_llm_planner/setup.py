from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cocoa_llm_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install prompts and grammar files
        (os.path.join('share', package_name, 'prompts'), 
            glob('prompts/*.txt') + glob('prompts/*.gbnf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='LLM-based action planner for Cocoa Speech drone control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'llm_planner_node = cocoa_llm_planner.llm_planner_node:main',
            'action_server_node = cocoa_llm_planner.action_server_node:main',
        ],
    },
)
