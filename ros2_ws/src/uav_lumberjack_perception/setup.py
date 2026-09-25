from setuptools import find_packages, setup

package_name = 'uav_lumberjack_perception'

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
    maintainer='ashuang',
    maintainer_email='ashuang1999@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_check = uav_lumberjack_perception.sensor_check_node:main',
            'red_branch_detector = uav_lumberjack_perception.red_branch_detector:main',
            'lidar_camera_projection = uav_lumberjack_perception.lidar_camera_projection:main',
        ],
    },
)
