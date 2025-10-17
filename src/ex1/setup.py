from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'ex1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabricio',
    maintainer_email='fabricio@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node1 = ex1.node1:main',
            'node2 = ex1.node2:main', 
            'node3 = ex1.node3:main', 
            'node4 = ex1.node4:main', 
            'node5 = ex1.node5:main',  
        ],
    },
)
