from setuptools import setup, find_packages

setup(
    name="pyrobo",
    version='0.1',
    description="Ben's personal robotics package with helpful functions for prototyping",
    author="Ben Kolligs",
    url='https://github.com/bkolligs/pyrobo.git',
    license='MIT',
    packages=find_packages(),
    zip_safe=False,
    install_requires=[
        'numpy',
        'matplotlib',
    ]
)
