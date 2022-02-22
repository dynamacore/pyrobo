#include <iostream>
#include "pybind11/pybind11.h"
#include "pybind11/eigen.h"
#include "Eigen/Dense"

namespace py = pybind11;

int add(int a, int b) {

	return a + b;
}

PYBIND11_MODULE(_pyrobo, m) {
	m.def("add", &add);
}