#ifndef _robo_utils_h_
#define _robo_utils_h_

#include <math.h>
#include <algorithm>

namespace utils
{

bool isApprox(double x, double y, double atol=0, double rtol=std::numeric_limits<double>::epsilon()); 

double norm(double x);

} // namespace utils
#endif //_robo_utils_h_ header