import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
		(os.path.join('share', package_name, 'sdf'), glob('sdf/*')),
		(os.path.join('share', package_name, 'meshes', 'iiwa7', 'collision'), glob('meshes/iiwa7/collision/*')),
		(os.path.join('share', package_name, 'meshes', 'iiwa7', 'visual'), glob('meshes/iiwa7/visual/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			"transform_publisher=robot_control.transform_publisher:main",
			"robot_managment=robot_control.robot_managment:main",
			"test_ik=robot_control.robotics_tools_test:main",
			"kinematic_solver=robot_control.kinematic_solver:main"
        ],
    },
)
