from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeon-ros2-humble-gazebo-wsl',
    maintainer_email='jeonwonje04@gmail.com',
    description='SLAM, frontier detection, exploration, coverage monitoring',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frontier_detector = amr_navigation.frontier_detector:main',
            'exploration_manager = amr_navigation.exploration_manager:main',
            'coverage_monitor = amr_navigation.coverage_monitor:main',
        ],
    },
)
