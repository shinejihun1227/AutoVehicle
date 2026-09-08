from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=["turn_signal_controller"], package_dir={"": "src"}))
