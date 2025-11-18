from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ollama_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='kimutai.workspace@gmail.com.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ollama_interface=ollama_ros2.ollama_store:main',
            "user=ollama_ros2.user_:main",
            'llm_mobile=ollama_ros2.llm_mobile_robotics:main'
        ],
    },
)
