from setuptools import find_packages
from setuptools import setup

setup(
    name='mirobot_package',
    version='0.0.0',
    packages=find_packages(
        include=('mirobot_package', 'mirobot_package.*')),
)
