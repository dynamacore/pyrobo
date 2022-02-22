import os, subprocess, platform, sys
from termcolor import colored
from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext
from pybind11.setup_helpers import Pybind11Extension, build_ext

"""
This is taken from this blog: https://www.benjack.io/2017/06/12/python-cpp-tests.html
Along with the PyBind11 docs: https://pybind11.readthedocs.io/en/stable/compiling.html#building-with-setuptools
And PyBind11 examples: https://github.com/pybind/python_example
And PyBind11 examples: https://github.com/pybind/cmake_example

Test that this works with `python setup.py develop`
"""

class CMakeExtension(Extension):
    def __init__(self, name, sources='') -> None:
        super().__init__(name, sources=[])
        self.sources = os.path.abspath(sources)

class CMakeBuild(build_ext):
    def run(self) -> None:
        try:
            print(colored("Building extension", "green"))
            # Check for the cmake version installed on the system
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: " 
                + ", ".join(e.name for e in self.extensions)
            )
        for ext in self.extensions:
            self.build_extension(ext)
    
    def build_extension(self, ext) -> None:
        extension_dir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name))
        )

        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY='+extension_dir,
            '-DPYTHON_EXECUTABLE='+sys.executable
        ]

        config = "Debug" if self.debug else "Release"
        build_args = ['--config', config]

        # These can be added only for non-windows systems
        cmake_args += ["-DCMAKE_BUILD_TYPE="+config]
        build_args += ["--", "-j2"]

        env_vars = os.environ.copy()
        env_vars["CXXFLAGS"] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env_vars.get("CXXFLAGS", ''),
            self.distribution.get_version()
        )

        if not os.path.exists(self.build_temp): os.makedirs(self.build_temp)

        subprocess.check_call(
            ['cmake', ext.sources] + cmake_args,
             cwd=self.build_temp, env=env_vars
        )
        subprocess.check_call(
            ['cmake', '--build', '.'] + build_args,
             cwd=self.build_temp
        )

        print()

setup(
    name="pyrobo",
    version='0.1',
    description="Robotics package to speed up prototyping of robotics systems",
    author="Ben Kolligs",
    author_email="benkolligs@gmail.com",
    url='https://github.com/bkolligs/pyrobo.git',
    license='MIT',
    # add extension module
    ext_modules=[CMakeExtension("robotics")],
    # add custom build exty command
    cmdclass=dict(build_ext=CMakeBuild),
    packages=find_packages(),
    zip_safe=False,
    install_requires=[
        'numpy',
        'matplotlib',
        'pybind11'
    ]
)
