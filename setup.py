from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_trajectory_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pete',
    maintainer_email='pete@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_smoother = path_trajectory_pkg.path_smoother:main',
            'path_smoother_clothoids = path_trajectory_pkg.path_smoother_clothoids:main',
            'trajectory_generator = path_trajectory_pkg.trajectory_generator:main',
            'trajectory_controller = path_trajectory_pkg.trajectory_controller:main',
            'trajectory_rpp_controller = path_trajectory_pkg.trajectory_rpp_controller:main',
        ],
    },
)
