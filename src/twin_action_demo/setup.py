from setuptools import find_packages, setup

package_name = 'twin_action_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
