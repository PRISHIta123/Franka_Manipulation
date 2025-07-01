from setuptools import setup, find_packages

setup(
    name="franka_bindings",
    version="0.1",
    packages=find_packages(),
    package_data={
        'franka_bindings': ['*.so'],
    },
) 