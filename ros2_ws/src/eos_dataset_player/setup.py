from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'eos_dataset_player'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'rosbags'],
    zip_safe=True,
    maintainer='EOS-SLAM',
    maintainer_email='user@example.com',
    description='EuRoC MAV dataset player for EOS-SLAM data pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dataset_player = eos_dataset_player.dataset_player:main',
            'bag_player = eos_dataset_player.bag_player:main',
        ],
    },
)
