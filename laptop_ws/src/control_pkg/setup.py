from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'control_pkg', 'launch'),
        glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	        'vibration_feature_node = control_pkg.vibration_feature_node:main',
            'fuzzy_speed_controller_node = control_pkg.fuzzy_speed_controller_node:main',
            'fuzzy_alpha_node = control_pkg.fuzzy_alpha_node:main',
            'cmd_vel_fusion_node = control_pkg.cmd_vel_fusion_node:main',
        ],
    },
)
