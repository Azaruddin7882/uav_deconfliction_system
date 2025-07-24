from setuptools import setup, find_packages

setup(
    name="uav_deconfliction",
    version="0.1",
    packages=find_packages(where="."),  # Finds all packages in root
    package_dir={"": "."},  # Root directory contains packages
    python_requires=">=3.10",
)