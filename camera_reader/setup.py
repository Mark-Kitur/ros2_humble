from setuptools import find_packages, setup

package_name = 'camera_reader'

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
            'cam_pub=camera_reader.camera_publisher:main',
            "distance_calc=camera_reader.distance_calculator:main",
            "aruco=camera_reader.auroc:main",
            "obj=camera_reader.objection_detection:main",
        ],
    },
)
