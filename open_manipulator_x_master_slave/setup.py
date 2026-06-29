import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'open_manipulator_x_master_slave'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergio Morales',
    maintainer_email='smorales@utec.edu.pe',
    description='Aplicación maestro-esclavo para dos OpenMANIPULATOR-X.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_slave_node = open_manipulator_x_master_slave.master_slave_node:main',
            'master_slave_panel = open_manipulator_x_master_slave.panel:main',
        ],
    },
)
