from setuptools import find_packages, setup

package_name = 'amr_launcher'

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
    description='Servo and spring actuation, ball delivery',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'delivery_sequencer = amr_launcher.delivery_sequencer:main',
        ],
    },
)
