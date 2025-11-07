from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share',package_name, 'config'), glob('config/*')),
        (os.path.join('share',package_name,'map'),glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='mark@todo.todo',
    description='Navigation robot package',
    license='MIT',
    entry_points={
        'console_scripts': [
            "nav_pose=navigation.nav_to_pose:main",
        ],
    },
)
