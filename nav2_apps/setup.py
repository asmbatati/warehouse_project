from setuptools import find_packages, setup
import os
from glob import glob
# from rosidl_python import generate_rosidl_types

package_name = 'nav2_apps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asmbatati',
    maintainer_email='asmalbatati@hotmail.com',
    description='Navigation applications for the warehouse project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'move_shelf_to_ship = nav2_apps.scripts.move_shelf_to_ship:main',
            # 'move_shelf_to_ship_real = nav2_apps.scripts.move_shelf_to_ship_real:main',
        ],
    },
)
