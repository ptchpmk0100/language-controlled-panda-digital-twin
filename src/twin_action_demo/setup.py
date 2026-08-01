from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'twin_action_demo'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files are only reachable through the share directory, so they
        # have to be installed explicitly.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # An ament_python package installs no data files by default, so
        # without this the launch file's share-directory lookup resolves to a
        # path that does not exist.
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khoi Minh Pham',
    maintainer_email='59853829+ptchpmk0100@users.noreply.github.com',
    description=(
        'ROS 2 action client/server foundation for the manipulator digital twin.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_joint_server = twin_action_demo.move_joint_server:main',
            'move_joint_client = twin_action_demo.move_joint_client:main',
            'move_arm_server = twin_action_demo.move_arm_server:main',
            'finger_state_publisher = '
            'twin_action_demo.finger_state_publisher:main',
        ],
    },
)
