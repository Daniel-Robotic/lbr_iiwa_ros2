from glob import glob
from setuptools import find_packages, setup

package_name = 'iiwa_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
		('share/' + package_name + '/config', glob('schemas/*.py')),
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
            'LBRiiwa_controller = iiwa_controller.LBRiiwa_controller:main',
			'positionPublisher = iiwa_controller.positionPublisher:main',
            "FRIcontroller = iiwa_controller.LBRiiwa_controller_real:main",
        ],
    },
)
