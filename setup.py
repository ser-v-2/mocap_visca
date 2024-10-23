import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'optitrack_visca'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ser_v',
    maintainer_email='ser_v@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_prueba = optitrack_visca.pub_prueba:main',
            'listener_prueba = optitrack_visca.sub_prueba:main',
            'listener = optitrack_visca.sub_optitrack_final:main',
            'combined_node = optitrack_visca.combined_node:main',
        ],
    },
)
