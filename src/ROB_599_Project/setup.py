from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ROB_599_Project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='Ryan.Suth01@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "move_pedestrian.py = ROB_599_Project.move_pedestrian:main",
            "mapping.py = ROB_599_Project.mapping:main",
            "find_dynamic.py = ROB_599_Project.find_dynamic:main",
            "find_dyanmic_2.py = ROB_599_Project.find_dynamic_2:main"
        ],
    },
)
