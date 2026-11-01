from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=["open_apero_ros"],
    package_dir={"": "python"},
)

setup(**setup_args)
