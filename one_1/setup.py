import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'one_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'data'), glob(os.path.join("data", "*.[pt]*"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='korol',
    maintainer_email='korolnastasia@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "follow_node = one_1.follow_node:main",
        ],
    },
)
