#include "robo_utils.h"

namespace utils
{
	
bool isApprox(double x, double y, double atol, double rtol) {
	return norm(x-y) <= std::max(atol, rtol*std::max(norm(x), norm(y)));
}

double norm(double x) {
	return sqrt(x*x);
}

} // namespace utils