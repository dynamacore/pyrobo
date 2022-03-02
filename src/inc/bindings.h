#ifndef _bindings_h_
#define _bindings_h_

#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/stl_bind.h"

/* Define opaque STL containers */
PYBIND11_MAKE_OPAQUE(std::pair<Eigen::Vector3d, double>);

namespace py = pybind11;
using namespace py::literals;

/**
 * Documentation for the bindings will be contained inside a python stubs file with extension *.pyi
 */
void quaternion_bind(py::module_ &m);

/**
 * We need an opaque type here to prevent copying back and forth between C++ and Python
 */
void opaque_pair(py::module_ &m);

#endif //_bindings_h_ header