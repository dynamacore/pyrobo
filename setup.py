from setuptools import setup

setup(
    name="robotics-prototyping",
    version='0.1',
    description="Ben's personal robotics package with helpful functions for prototyping",
    author="Ben Kolligs",
    url='https://github.com/bkolligs/robotics-prototyping.git',
    license='MIT',
    packages=['robotics'],
    zip_safe=False,
    install_requires=[
        'numpy'
    ]
)