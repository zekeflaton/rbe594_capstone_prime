import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'urdf', 'robot_parts'), glob('urdf/robot_parts/*')),
        (os.path.join('share', package_name, 'urdf', 'sensors'), glob('urdf/sensors/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xarco')),
        (os.path.join('share', package_name), glob('world/*')),
        (os.path.join('share', package_name), glob('rviz/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='chrisjd91@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
