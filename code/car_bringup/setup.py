from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'car_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedarklord',
    maintainer_email='miracleyt404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_to_ackermann = car_bringup.twist_to_ackermann:main',
            'web_bridge = car_bringup.web_bridge:main',
            'scan_filter = car_bringup.scan_filter:main',
        ],
    },
)
