from setuptools import setup


package_name = 'controller_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arthur Nunes',
    maintainer_email='arthurhdn7@hotmail.com',
    description='ROS 2 node that streams a full-state command to a Crazyflie through Crazyswarm2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_node = controller_pkg.controller_node:main',
        ],
    },
)