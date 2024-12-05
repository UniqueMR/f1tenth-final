from setuptools import find_packages
from setuptools import setup

setup(
    name='final_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('final_pkg', 'final_pkg.*')),
)
