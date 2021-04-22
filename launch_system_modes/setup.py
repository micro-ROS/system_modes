from setuptools import setup

package_name = 'launch_system_modes'

setup(
    name=package_name,
    version='0.7.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arne Nordmann',
    maintainer_email='arne.nordmann@de.bosch.com',
    description='System modes specific extensions to the launch tool.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
