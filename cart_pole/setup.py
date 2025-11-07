from setuptools import setup
import os
from glob import glob

package_name = 'cart_pole'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS 2 package registration
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Include launch, config, and urdf directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mark Kimutai Kitur',
    maintainer_email='kiturmark@gmail.com',
    description='Cart Pole simulation package for Gazebo Classic with ROS 2 Control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cartpole_reset=cart_pole.cart_pole_reset:main",
            "cartpole_training=cart_pole.cartole_training:main",
            "cartpole_pred=cart_pole.use_ppo:main",
        ],
    },
)
