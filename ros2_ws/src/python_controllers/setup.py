from distutils.core import setup
from catkin_pkg.python_setup import generate_distuilts_setup

# from setuptools import setup

package_name = 'python_controllers'

d = generate_distuilts_setup()
d["packages"] = ["python_controllers"]
d["package_dir"] = {"": "src"}
setup(**d)

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='zeke',
#     maintainer_email='zekeeflaton@gmail.com',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
