from setuptools import find_packages, setup

package_name = 'amr_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeon-ros2-humble-gazebo-wsl',
    maintainer_email='jeonwonje04@gmail.com',
    description='Legacy lab exercise nodes for testing and debugging',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'r2auto_nav = amr_utils.r2auto_nav:main',
            'r2mover = amr_utils.r2mover:main',
            'r2moverotate = amr_utils.r2moverotate:main',
            'r2scanner = amr_utils.r2scanner:main',
            'r2occupancy = amr_utils.r2occupancy:main',
            'r2occupancy2 = amr_utils.r2occupancy2:main',
        ],
    },
)
