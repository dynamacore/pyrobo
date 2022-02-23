#include "bindings.h"
#include "quaternion.h"

void quaternion_bind(py::module_ &m) {
	py::class_<Quaternion>(m, "_Quaternion")
		.def(py::init<double, double, double, double>(),
			"w"_a=0, "x"_a=0, "y"_a=0, "z"_a=0)
		.def(py::init<Eigen::Vector3d>())
		.def("_w"          , &Quaternion::w)
		.def("_x"          , &Quaternion::x)
		.def("_y"          , &Quaternion::y)
		.def("_z"          , &Quaternion::z)
		.def("_data"       , &Quaternion::data)
		.def("_inv"        , &Quaternion::inv)
		.def("_norm"       , &Quaternion::norm)
		.def("_adjoint"    , &Quaternion::adjoint)
		.def("_approx"     , &Quaternion::isApprox)
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
		// Define the multiplication overloading for a vector
		.def("__mul__", [](Quaternion&q, Eigen::Vector3d & v){
			return q*v;
		})
		;
}