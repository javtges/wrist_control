from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_tracking'

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
    maintainer='amber',
    maintainer_email='amberhandal24@gmail.com',
    description='Baseplate camera + AprilTag tracking of the HSA wrist platform',
    license='MIT',
    entry_points={
        'console_scripts': [
            'platform_pose = camera_tracking.platform_pose:main',
            'tag_monitor = camera_tracking.tag_monitor:main',
            'plate_check = camera_tracking.plate_check:main',
            'plate_guard = camera_tracking.plate_guard:main',
        ],
    },
)
