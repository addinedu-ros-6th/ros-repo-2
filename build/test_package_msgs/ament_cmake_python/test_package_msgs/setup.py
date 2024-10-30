from setuptools import find_packages
from setuptools import setup

setup(
    name='test_package_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('test_package_msgs', 'test_package_msgs.*')),
)
