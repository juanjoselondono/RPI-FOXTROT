from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ybeb_2602_zulu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi5master',
    maintainer_email='rpi5master@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Ensure these executable names perfectly match what is written in your launch files
            'ybeb_node.py = ybeb_2602_zulu.ybeb_node:main',
            'aeb_ttc.py = ybeb_2602_zulu.aeb_ttc:main',
            'supervisor.py = ybeb_2602_zulu.supervisor:main'
        ],
    },
)
