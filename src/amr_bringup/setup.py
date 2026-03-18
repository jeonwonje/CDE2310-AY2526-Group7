from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeon-ros2-humble-gazebo-wsl',
    maintainer_email='jeonwonje04@gmail.com',
    description='Mission orchestration and launch files',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_controller = amr_bringup.mission_controller:main',
        ],
    },
)
