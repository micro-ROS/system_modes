from setuptools import find_packages
from setuptools import setup

package_name = 'launch_system_modes'

setup(
    name=package_name,
    version='0.7.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'launch_ros',
        'lifecycle_msgs',
        'setuptools',
        'system_modes_msgs',
        ],
    zip_safe=True,
    maintainer='Arne Nordmann',
    maintainer_email='arne.nordmann@de.bosch.com',
    keywords=['ROS', 'launch', 'system modes'],
    description='System modes specific extensions to launch_ros.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
