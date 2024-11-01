from setuptools import find_packages, setup

package_name = 'test_package'

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
    maintainer='minibot',
    maintainer_email='minibot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_publisher_udp = test_package.aruco_publisher_udp:main',
            'camera_subscriber = test_package.camera_subscriber:main',

            'distance_publisher = test_package.distance_publisher:main',
            'distance_subscriber = test_package.distance_subscriber:main',

            'aruco_publisher= test_package.aruco_publisher:main',
            'aruco_subscriber= test_package.aruco_subscriber:main',

        ],
    },
)
