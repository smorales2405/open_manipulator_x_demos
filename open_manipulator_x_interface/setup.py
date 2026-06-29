import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'open_manipulator_x_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergio Morales',
    maintainer_email='smorales@utec.edu.pe',
    description='Interfaz de taller PyQt5 para el OpenMANIPULATOR-X real.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_bridge = open_manipulator_x_interface.robot_bridge:main',
            'interface_gui = open_manipulator_x_interface.interface_gui:main',
        ],
    },
)
