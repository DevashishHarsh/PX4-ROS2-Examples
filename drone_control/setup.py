from setuptools import find_packages, setup

package_name = 'drone_control'

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
    maintainer='daisuke',
    maintainer_email='daisuke@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drone_formation_change = drone_control.drone_formation_change:main",
            "drone_formation_control = drone_control.drone_formation_control:main",
            "drone_formation = drone_control.drone_formation:main",
            "drone_leader = drone_control.drone_leader:main",
            "drone_manual = drone_control.drone_manual:main",
            "drone_select = drone_control.drone_select:main",
        ],
    },
)
