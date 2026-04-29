from glob import glob
import os

from setuptools import setup

package_name = 'central_controller'


setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.nodes',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbam',
    maintainer_email='maintainer@example.com',
    description='Central ROS 2 controller node for LIMO observation acknowledgment tests.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'central_controller_node = central_controller.nodes.central_controller_node:main',
        ],
    },
)
