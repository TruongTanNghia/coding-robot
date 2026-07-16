from setuptools import setup
import os
from glob import glob

package_name = 'strawberry_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Nghia',
    maintainer_email='nghia@example.com',
    description='Xe hai dau: base bridge + ne vat can bang LiDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_bridge = strawberry_bot.base_bridge:main',
            'obstacle_avoider = strawberry_bot.obstacle_avoider:main',
        ],
    },
)
