from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mipi_camera_simple'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Simple MIPI Camera package for RDK robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = mipi_camera_simple.camera_publisher:main',
            'camera_viewer = mipi_camera_simple.camera_viewer:main',
        ],
    },
)
