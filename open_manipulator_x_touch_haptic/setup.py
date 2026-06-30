import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'open_manipulator_x_touch_haptic'

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
    description='Teleoperación cinemática del OpenMANIPULATOR-X con el Geomagic Touch.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'touch_haptic_node = open_manipulator_x_touch_haptic.touch_haptic_node:main',
            'touch_haptic_panel = open_manipulator_x_touch_haptic.panel:main',
            'virtual_touch = open_manipulator_x_touch_haptic.virtual_touch:main',
        ],
    },
)
