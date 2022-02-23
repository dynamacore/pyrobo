#include "bindings.h"
#include "quaternion.h"

void quaternion_bind(py::module_ &m) {
	py::class_<Quaternion>(m, "Quaternion")
		.def(py::init<double, double, double, double>(),
			"w"_a=0, "x"_a=0, "y"_a=0, "z"_a=0)
		.def("w"          , &Quaternion::w)
		.def("x"          , &Quaternion::x)
		.def("y"          , &Quaternion::y)
		.def("z"          , &Quaternion::z)
		.def("data"       , &Quaternion::data)
		.def("inv"        , &Quaternion::inv)
		.def("norm"       , &Quaternion::norm)
		.def("adjoint"    , &Quaternion::adjoint)
		.def("approx"     , &Quaternion::isApprox)
		.def("__repr___"  , &Quaternion::toRepr)
		.def("__str__"    , &Quaternion::toString)
		.def(double() * py::self)
		.def(py::self * double())
		.def(py::self * py::self)
		.def(double() + py::self)
		.def(py::self + double())
		.def(py::self + py::self)
		.def(double() - py::self)
		.def(py::self - double())
		.def(py::self - py::self)
		.def(double() / py::self)
		.def(py::self / double())
		.def(py::self / py::self)
		;
}