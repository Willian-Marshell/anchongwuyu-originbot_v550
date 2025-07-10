from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'clip_snip'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'Model'), glob('Model/*.bin')),
        (os.path.join('share',package_name,'launch'), glob(os.path.join('launch','*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clip_detector = clip_snip.clip_detector:main',
            'motion_controller = clip_snip.motion_controller:main',
            'magnet_controller = clip_snip.magnet_controller:main',
        ],
    },
)
