from setuptools import setup
import os
from glob import glob

package_name = 'slam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        #(os.path.join('share', package_name, 'slam_pkg/nodes'), glob('slam_pkg/nodes/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='SLAM package for diffRobot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'odom_tf_broadcaster = slam_pkg.nodes.odom_tf_broadcaster:main',
        ],
    },
)