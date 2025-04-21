import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'crazyflie_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardo',
    maintainer_email='americogomes1@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "crazyflie_aruco_control = crazyflie_aruco.crazyflie_aruco_control:main",
            "control_crazyflie_lifecycle = crazyflie_aruco.control_aruco_movement:main",
            "test_crazyflie = crazyflie_aruco.test_crazyflie:main",
        ],
    },
)
