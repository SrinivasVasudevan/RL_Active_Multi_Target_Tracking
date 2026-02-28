from glob import glob
import os
from setuptools import setup

package_name = 'mbam_gazebo_tracking'


def files_in(path_pattern):
    return [f for f in glob(path_pattern, recursive=True) if os.path.isfile(f)]


setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        f'{package_name}.core',
        f'{package_name}.nodes',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files_in('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), files_in('worlds/*')),
        (os.path.join('share', package_name, 'models', 'agent_bot'), files_in('models/agent_bot/*')),
        (os.path.join('share', package_name, 'models', 'target_bot'), files_in('models/target_bot/*')),
        (os.path.join('share', package_name, 'rviz'), files_in('rviz/*')),
        (os.path.join('share', package_name, 'config'), files_in('config/*')),
        (os.path.join('share', package_name, 'checkpoints'), files_in('checkpoints/*')),
        (os.path.join('share', package_name, 'scripts'), files_in('scripts/*')),
        (os.path.join('share', package_name, 'docs'), files_in('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbam',
    maintainer_email='maintainer@example.com',
    description='ROS 2 Gazebo + RViz testing package for model-based active mapping with attention policy control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'episode_runner = mbam_gazebo_tracking.nodes.episode_runner_node:main',
        ],
    },
)
