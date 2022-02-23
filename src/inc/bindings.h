#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"
#include "pybind11/operators.h"

namespace py = pybind11;
using namespace py::literals;

void quaternion_bind(py::module_ &m);