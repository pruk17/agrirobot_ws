from glob import glob
import os
from setuptools import setup

package_name = 'agrirobot_core'

setup(
    # Basic package info
    name=package_name,  # The name of the ROS 2 Python package
    version='0.0.0',    # Package version

    # Now we have a proper Python package folder
    packages=[package_name],  # Include the package folder so Python recognizes it as a module
    # py_modules is no longer needed because the scripts are inside the package folder

    install_requires=['setuptools'],  # Dependencies for building/installing
    zip_safe=True,  # Package can be installed as a .zip or extracted
    maintainer='Pruk',  # Your name as maintainer
    maintainer_email='chaiyapruk6494@gmail.com',  # Maintainer email
    description='Motor control publisher for ROS 2',  # Short description
    license='MIT',  # License type

    data_files=[
        # Standard ROS2 resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Copy package.xml to share
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],

    # Entry points define console scripts for Python nodes
    entry_points={
        'console_scripts': [
            # Create a command to run DiffdrivePublisher node with ros2 run
            # The module path now includes the package name: agrirobot_core.DiffdrivePublisher
            'diffdrive_publisher = agrirobot_core.DiffdrivePublisher:main',  # Must point to main() in the file
            'joystick_publisher = agrirobot_core.JoyStickPublisher:main',
            # Additional Python nodes can be added here if created
            # Example: 'NewNode = agrirobot_core.NewNode:main'
        ],
    },
)
