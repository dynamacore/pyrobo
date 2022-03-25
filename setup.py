import os

from glob import glob
from pybind11.setup_helpers import Pybind11Extension
from setuptools import find_packages, setup
from termcolor import colored
import platform

"""
Along with the PyBind11 docs: https://pybind11.readthedocs.io/en/stable/compiling.html#building-with-setuptools
And PyBind11 examples: https://github.com/pybind/python_example

Test that this works with `python setup.py develop`
"""
# Use G++ to compile C++
os.environ["CC"] = "g++"
if 

# Places to look for eigen on the machine
candidate_paths = [
    os.environ.get("EIGEN_INCLUDE_DIR", "/usr/include/eigen3"),
    "/usr/local/include/eigen3"
]

eigen_path = iter(candidate_paths)
for eigen_path in candidate_paths:
    print(f"Looking for Eigen at the path: {eigen_path}")
    if not os.path.exists(eigen_path):
        print(colored(f"Could not find Eigen3 at the path: {eigen_path}\n"), 'yellow')

        USE_CONDA_INCLUDE_FOR_EIGEN = True
    else:
        USE_CONDA_INCLUDE_FOR_EIGEN = False
        break

if USE_CONDA_INCLUDE_FOR_EIGEN:
    if "CONDA_PREFIX" in os.environ:
        conda_path = os.environ.get("CONDA_PREFIX")
        eigen_path = conda_path + "/include/eigen3"
        print(colored(f"Using conda environment defined here to look for Eigen'{eigen_path}'", 'green'))
    else:
        raise RuntimeError(colored("Make sure to activate your conda environment!", 'red'))

if not os.path.exists(eigen_path):
    raise RuntimeError(colored("\nEigen3 is not installed in the current environment! Please install it with one of the following options: \n\t1. sudo apt install libeigen3-dev\n\t2. conda install -c conda-forge eigen\n\t3. Download the git repository as described here: https://eigen.tuxfamily.org/dox/GettingStarted.html", 'red'))

print(colored(f"Eigen found at '{eigen_path}'!", 'green'))

include_dirs = [eigen_path, 'src/inc', 'src/bindings']
ext_modules = [ 
    Pybind11Extension(
        "_pyrobo",
        sorted(
            [
                "src/quaternion.cpp",
                "src/robo_utils.cpp",
                # Bindings live in their own folders 
                "src/bindings/bindings.cpp",
                "src/bindings/quaternion_bind.cpp"
            ]
        ), 
        cxx_std=17,
        language='c++',
        include_dirs=include_dirs
    )
]

setup(
    name="pyrobo",
    version='0.1',
    description="Robotics package to speed up prototyping of robotics systems",
    author="Ben Kolligs",
    author_email="benkolligs@gmail.com",
    url='https://github.com/bkolligs/pyrobo.git',
    license='MIT',
    # add extension module
    ext_modules=ext_modules,
    packages=find_packages(),
    extras_require={"test": "pytest"},
    zip_safe=False,
    install_requires=[
        'numpy',
        'matplotlib',
    ],
    # Type data for stub files
    package_data= {
        'robotics': [
            'py.typed',
            'quaternion.pyi'
            ]
    }
)
