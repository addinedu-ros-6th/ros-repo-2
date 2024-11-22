import glob
import os
from setuptools import find_packages, setup

package_name = 'servee_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.*'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_trees = servee_robot.robot_trees:main',
            'battery_state_pub = servee_robot.battery_state_pub:main',
            'servee_goal_test_node = servee_robot.receive_goal_test:main',
            'picam_processor = servee_robot.picam_processor:main',
        ],
    },
)
