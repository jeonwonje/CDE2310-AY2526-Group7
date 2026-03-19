from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeon-ros2-humble-gazebo-wsl',
    maintainer_email='jeonwonje04@gmail.com',
    description='Custom navigation: Dijkstra target finding, A* wall penalty, cluster movement',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'auto_nav = amr_nav.auto_nav:main',
            'coverage_monitor = amr_nav.auto_nav:main_coverage',
        ],
    },
)
