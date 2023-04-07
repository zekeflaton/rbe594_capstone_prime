# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup
import os
from glob import glob
from setuptools import setup

package_name = 'python_controllers'

# d = generate_distutils_setup(
#     packages = ["python_controllers"],
#     scripts=["scripts"],
#     package_dir = {"": "src"}
# )
# setup(**d)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name+'/scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ( os.path.join('share', package_name, 'src'), glob('src/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeke',
    maintainer_email='zekeeflaton@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_test = scripts.commander_test:main',
            'compute_time_analysis = scripts.compute_time_analysis:main',
            'deadlock_vs_robots_counts_analysis = scripts.deadlock_vs_robots_counts_analysis:main'
            'path_efficiency_analysis = scripts.path_efficiency_analysis:main',
            'run_analysis_sim = scripts.run_analysis_sim:main',
            'run_orchestrator = scripts.run_orchestrator:main',
            'save_tags_to_pickle_file = scripts.save_tags_to_pickle_file:main',
        ],
    },
)
