from setuptools import find_packages, setup

package_name = 'motor_driver_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rbrv',
    maintainer_email='rbrv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_teleop_subscriber = motor_driver_test.motor_teleop_subscriber:main',
            'mecanum_driver_node = motor_driver_test.mecanum_driver_node:main',
        ],
    },
)
