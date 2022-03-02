#include "bindings.h"
#include "quaternion.h"

void quaternion_bind(py::module_ &m) {
	py::class_<Quaternion>(m, "Quaternion")
		.def(py::init<double, double, double, double>(),
			"w"_a=0, "x"_a=0, "y"_a=0, "z"_a=0)
		.def(py::init<Eigen::Vector3d>(),
			"p"_a)
		.def(py::init<Eigen::Vector3d, double>(), 
			"axis"_a, "angle"_a)
		.def(py::init<double, double, double>(),
			"roll"_a, "pitch"_a, "yaw"_a)
		.def("w"          , &Quaternion::w)
		.def("x"          , &Quaternion::x)
		.def("y"          , &Quaternion::y)
		.def("z"          , &Quaternion::z)
		.def("data"       , &Quaternion::data)
		.def("inv"        , &Quaternion::inv)
		.def("norm"       , &Quaternion::norm)
		.def("adj"    , &Quaternion::adjoint)
		.def("approx"     , &Quaternion::isApprox,
			"other"_a, "atol"_a=0, "rtol"_a=std::sqrt(std::numeric_limits<double>::epsilon()))
		.def("rotation", &Quaternion::isRotation, 
			"atol"_a=1E-12)
		.def("to_axis_ang", &Quaternion::toAxisAngle)
		.def("to_matrix", &Quaternion::toRotationMatrix)
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
		.def(py::self == py::self)
		// Define the multiplication overloading for a vector
		.def("__mul__", [](Quaternion&q, Eigen::Vector3d & v){
			return q*v;
		})
		;
}