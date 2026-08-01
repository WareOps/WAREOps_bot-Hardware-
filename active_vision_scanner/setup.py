import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'active_vision_scanner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinav',
    maintainer_email='ironman18122004@gmail.com',
    description='Active Vision Rack Scanning System with OpenCV and servo control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rack_scanner = active_vision_scanner.rack_scanner_node:main',
            'stitched_rack_scanner = active_vision_scanner.stitched_rack_scanner_node:main',
            'esp32_bridge = active_vision_scanner.esp32_serial_bridge:main',
        ],
    },
)
