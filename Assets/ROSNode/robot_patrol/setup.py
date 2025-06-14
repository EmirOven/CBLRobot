from setuptools import setup

package_name = 'robot_patrol'

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
    maintainer='ubuntuhost',
    maintainer_email='bugra.karac@gmail.com',
    description='',
    license='',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_patrol_node = robot_patrol.robot_patrol_node:main',
    ],
},
)
