from setuptools import find_packages, setup

package_name = 'mecanum_driver_pid'

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
            'mecanum_driver_node = mecanum_driver_pid.mecanum_driver_node_final:main',
            'mecanum_driver_tf = mecanum_driver_pid.mecanum_driver_node_tf:main',
            'pwm_adjustable = mecanum_driver_pid.pwm_adjustable_node:main',
        ],
    },
)
