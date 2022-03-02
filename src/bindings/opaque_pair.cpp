#include "bindings.h"

void opaque_pair(py::module_ &m) {
	py::class_<std::pair<Eigen::Vector3d, double>>(m, "AxisAnglePair")
	.def(py::init<>());
}