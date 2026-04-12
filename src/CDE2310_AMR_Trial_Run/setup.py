import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'CDE2310_AMR_Trial_Run'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.lua'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kumaresan',
    maintainer_email='kugapryank@gmail.com',
    description='Mission coordination, exploration, and navigation for CDE2310 AMR',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mission_coordinator = CDE2310_AMR_Trial_Run.mission_coordinator_v3:main',
            'mission_coordinator_prototype = CDE2310_AMR_Trial_Run.mission_coordinator_prototype:main',
            'mission_coordinator_custom_explorer = CDE2310_AMR_Trial_Run.mission_coordinator_custom_explorer:main',
            'search_server = CDE2310_AMR_Trial_Run.search_stations:main',
            'static_station = CDE2310_AMR_Trial_Run.static_station:main',
            'apriltag_detector = CDE2310_AMR_Trial_Run.apriltag_detector:main',
            'docking_server = CDE2310_AMR_Trial_Run.docker:main',
            'delivery_server = CDE2310_AMR_Trial_Run.delivery_server:main',
            'launcher_node = CDE2310_AMR_Trial_Run.launcher_node:main',
            'rpi_shooter_node = CDE2310_AMR_Trial_Run.rpi_shooter_node:main',
        ],
    },
)
