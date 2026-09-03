from setuptools import find_packages, setup

package_name = 'node_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['app'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'textual>=8.2,<9', 'PyYAML>=6,<7', 'ros2cli'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Terminal command center for inspecting and controlling ROS 2 systems.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_center = app:main',
            'simple_pub = node_viewer.simple_pub:main',
            'simple_sub = node_viewer.simple_sub:main',
        ],
        'ros2cli.command': [
            'cmd_center = node_viewer.ros2cli.command.cmd_center:CmdCenterCommand',
        ],
    },
)
