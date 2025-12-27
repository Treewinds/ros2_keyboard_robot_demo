from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'keyboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/keyboard_control']),
        ('share/keyboard_control', ['package.xml']),
        (os.path.join('share', 'keyboard_control', 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', 'keyboard_control', 'worlds'),
        glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='treewinds',
    maintainer_email='Jrh150583@163.com',
    description='Keyboard control for Gazebo car',
    license='TODO',
    entry_points={
        'console_scripts': [
            'keyboard_car = keyboard_control.keyboard_car_node:main',
            'auto_patrol = keyboard_control.auto_patrol:main',
            'auto_square_odom = keyboard_control.auto_square_odom:main',
            'empty_node = keyboard_control.empty_node:main',
        ],
    },
)

