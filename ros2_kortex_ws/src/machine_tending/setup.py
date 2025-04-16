from setuptools import setup

package_name = 'machine_tending'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mo',
    maintainer_email='mampane@olin.edu',
    description='ROS2 package for controlling Gen3 7DOF Kinova arm for Olin College Tormach work cell',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'relay_interface_node = machine_tending.relay_interface:main',
        ],
    },
)
