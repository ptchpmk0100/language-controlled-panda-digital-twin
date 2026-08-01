from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'twin_moveit_scripts'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khoi Minh Pham',
    maintainer_email='59853829+ptchpmk0100@users.noreply.github.com',
    description=(
        'Scripted motion primitives for the Panda, built on moveit_py.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to = twin_moveit_scripts.move_to:main',
            'llm_move = twin_moveit_scripts.llm_move:main',
        ],
    },
)
