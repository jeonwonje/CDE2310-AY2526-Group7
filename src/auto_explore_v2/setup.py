import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auto_explore_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # FIX: Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # FIX: Include all config files (yaml)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kuga',
    maintainer_email='kugapryank@gmail.com',
    description='Frontier exploration and scoring for CDE2310 project',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'find_frontiers = auto_explore_v2.find_frontiers:main',
            'score_and_post = auto_explore_v2.score_and_post:main',
            'wander = auto_explore_v2.wander:main',
        ],
    },
)
