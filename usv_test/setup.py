from setuptools import setup
import os
from glob import glob

package_name = 'usv_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alexandernrobb@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_test = usv_test.velocity_test:main',
            'heading_test = usv_test.heading_test:main',
            'square_test = usv_test.square_test:main',
            'imu_quat_to_euler = usv_test.imu_quat_to_euler:main'
        ],
    },
)
