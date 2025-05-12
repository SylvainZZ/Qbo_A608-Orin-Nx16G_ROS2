from setuptools import find_packages
from setuptools import setup

setup(
    name='qbo_msgs',
    version='0.1.1',
    packages=find_packages(
        include=('qbo_msgs', 'qbo_msgs.*')),
)
