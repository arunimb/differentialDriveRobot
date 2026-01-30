from setuptools import setup
import os
from glob import glob

package_name = 'diffRobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files - all Python launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py') + glob('launch/*.launch.py')),
        
        # URDF files - all URDF and XACRO files
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.urdf*') + glob('urdf/*.xacro') + glob('urdf/*.xacro*')),
        
        # Config files - all YAML and config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') + glob('config/*.yml') + glob('config/*.cfg')),
        
        # RViz config files
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
        
        # World files - all Gazebo world files
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world') + glob('worlds/*.sdf')),
        
        # Map files - all map files
        (os.path.join('share', package_name, 'map'), 
            glob('map/*.pgm') + glob('map/*.png') + glob('map/*.jpg') + 
            glob('map/*.yaml') + glob('map/*.yml')),
        
        # Meshes - all mesh files (common formats)
        (os.path.join('share', package_name, 'meshes'), 
            glob('meshes/*.STL') + glob('meshes/*.stl') + 
            glob('meshes/*.dae') + glob('meshes/*.obj') + 
            glob('meshes/*.ply')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Mazzetto',
    maintainer_email='lucasrmazzetto@gmail.com',
    description='A ROS 2 package for simulating a simple differential drive robot with Gazebo, Nav2, and SLAM.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your node entry points here if you create any Python nodes later
        ],
    },
)