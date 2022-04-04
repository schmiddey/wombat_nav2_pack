from setuptools import setup

package_name = 'wombat_waypoint'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m1ch1',
    maintainer_email='m4ffle@googlemail.com',
    description='Nodes for ez handling of nav2 waypoint navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'waypoint_clicker_node = wombat_waypoint.waypoint_clicker:main',
          'waypoint_executer_node = wombat_waypoint.waypoint_executer:main',
        ],
    },
)


