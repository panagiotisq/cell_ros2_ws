import os
from glob import glob
from setuptools import setup

package_name = 'cell_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Panagiotis Kiousis',
    maintainer_email='pkious2003@gmail.com',
    description='CELL ROS 2 relocalization node (event camera in a prior LiDAR map).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # `ros2 run cell_localization localizer_node`
            'localizer_node = cell_localization.localizer_node:main',
        ],
    },
)
