import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'elpan_manual'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hisyam',
    maintainer_email='hisyam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32f4 = elpan_manual.STM32SerElpan:main',
            'stickconnect = elpan_manual.stickconnect:main',
            'dataolah = elpan_manual.dataolah:main',
            'camyolo = elpan_manual.camyolo:main'
        ],
    },
)
