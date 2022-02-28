# Contributing to the Robotics Prototyping Package

Python style: 
	Functions and Variables: snake_case
	Classes: PascalCase

C++ style: 
	Functions: camelCase
	Classes: PascalCase
	Variables: snake_case

# Include Stub files alongside C++ classes for intellisense

This allows the intellisense provider or LSP find type hints and docstrings for the C++ extension code. 

For example if the python bindings are as follows: 
```C++

namespace py=pybind11

int add(int i, int j) {
	return i + j;
}

PYBIND11_MODULE(_sample, m){
	m.def("add", &add, "adding function", py::arg("i"), py::arg("j"))
}
```

Then there should be an associated `add.py` and `add.pyi` file that actually import this code into the package. The `*.pyi` file contains "prototypes" for each function so that intellisense can use it. For this example our files would look like:

_add.py_
```python
from _sample import add
```

_add.pyi_
```python
def add(i: int, j: int) -> int: 
	"""
	Adds two integers together
	"""
	...
```