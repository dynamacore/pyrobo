#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"
#include "pybind11/operators.h"

namespace py = pybind11;
using namespace py::literals;

/**
 * Documentation for the bindings will be contained inside a python stubs file with extension *.pyi
 */
void quaternion_bind(py::module_ &m);