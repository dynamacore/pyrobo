#include "bindings.h"
#include "quaternion.h"

void quaternion_bind(py::module_ &m) {
	py::class_<Quaternion>(m, "Quaternion")
		.def(py::init<double, double, double, double>(),
			"w"_a=0, "x"_a=0, "y"_a=0, "z"_a=0)
		.def(double() * py::self);
}