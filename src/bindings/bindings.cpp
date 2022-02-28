#include "bindings.h"

namespace py = pybind11;

PYBIND11_MODULE(_pyrobo, m){
	quaternion_bind(m);
}