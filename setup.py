from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wall_follower_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models/urdf'), glob('models/urdf/*')),
        (os.path.join('share', package_name, 'models/meshes'), glob('models/meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parveez',
    maintainer_email='parveezbanu.s@gmail.com',
    description='ROS 2 Gazebo-based obstacle avoidance robot simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'wall_follower_bot_node = wall_follower_bot.wall_follower_bot_node:main',
    ],
},

)

