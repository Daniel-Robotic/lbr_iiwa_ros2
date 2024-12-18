from glob import glob
from setuptools import find_packages, setup

package_name = 'lbr_intel_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rnf',
    maintainer_email='rnf@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			"stream_camera = lbr_intel_camera.camera_stream:main",
			"calibration = lbr_intel_camera.calibration:main", 
        ],
    },
)
