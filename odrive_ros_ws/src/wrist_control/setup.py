from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wrist_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'doc'), glob('doc/*.md')),
        # ros2 run looks in lib/<package>/ — setuptools 84 ignores [install] install_scripts
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='jpavtges48@gmail.com',
    description='HSA wrist ODrive control node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hsa_odrive_control = wrist_control.hsa_odrive_control:main',
            'joint_commander_gui = wrist_control.joint_commander_gui:main',
            'odrive_position_monitor = wrist_control.odrive_position_monitor:main',
            'spacemouse_commander = wrist_control.spacemouse_commander:main',
            'wrist_state_publisher = wrist_control.wrist_state_publisher:main',
        ],
    },
)
