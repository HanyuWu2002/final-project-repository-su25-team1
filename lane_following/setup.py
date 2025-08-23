from setuptools import setup
import os
from glob import glob

package_name = 'lane_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'ucsd_robocar_nav2_pkg'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_following_executable = lane_following.lane_following:main'
        ],
    },
)
