#include "quaternion.h"

Quaternion::Quaternion() : w_{0}, x_{0}, y_{0}, z_{0} {}
Quaternion::Quaternion(double w, double x, double y, double z) : w_{w}, x_{x}, y_{y}, z_{z} {}

Quaternion::Quaternion(Eigen::Vector3d p) {
	w_ = 0.0;
	x_ = p(0)/p.norm();
	y_ = p(1)/p.norm();
	z_ = p(2)/p.norm();
}

Quaternion::~Quaternion(){}

std::string Quaternion::toString() const {
	std::stringstream ss;
	ss << "[" << w_ << ", " << x_ << "i, " << y_ << "j, " << z_<< "k]";
	return ss.str();
}

std::string Quaternion::toRepr() const {
	std::stringstream ss;
	ss << "Quaternion(" << w_ << ", " << x_ << ", " << y_ << ", " << z_<< ")";
	return ss.str();
}

Quaternion Quaternion::inv() const {
	return adjoint()/(norm()*norm());
}

Quaternion Quaternion::adjoint() const {
	return Quaternion(w(), x(), y(),z());
}

double Quaternion::norm() const {
	return sqrt(w()*w() + x()*x() + y()*y() + z()*z());
}

bool Quaternion::isApprox(Quaternion other, double atol, double rtol) {
	return 	utils::isApprox(w(), other.w(), atol=atol, rtol=rtol) and
			utils::isApprox(x(), other.x(), atol=atol, rtol=rtol) and
			utils::isApprox(y(), other.y(), atol=atol, rtol=rtol) and
			utils::isApprox(z(), other.z(), atol=atol, rtol=rtol);
}

/**
 * Quaternion number operator overloads, including the member overloads and the friend overloads together
 */
Quaternion Quaternion::operator+(const double a) const {return Quaternion(w()+a, x(), y(), z());}
Quaternion operator+(const double a, const Quaternion &q) {return Quaternion(q.w()+a, q.x(), q.y(), q.z());}

Quaternion Quaternion::operator-(const double a) const {return Quaternion(w()-a, x(), y(), z());}
Quaternion operator-(const double a, const Quaternion &q) {return Quaternion(q.w()-a, q.x(), q.y(), q.z());}
Quaternion Quaternion::operator*(const double a) const {return Quaternion(w()*a, x()*a, y()*a, z()*a);}
Quaternion operator*(const double a, const Quaternion &q) {return Quaternion(q.w()*a, q.x()*a, q.y()*a, q.z()*a);}

Quaternion Quaternion::operator/(const double a) const {return Quaternion(w()/a, x()/a, y()/a, z()/a);}
Quaternion operator/(const double a, const Quaternion &q) {Quaternion qTemp = q.inv(); return a*qTemp;}

/**
 * Quaternion Quaternion operator overloads
 */
Quaternion Quaternion::operator*(const Quaternion &q) const {
	double new_w = w()*q.w() - x()*q.x() - y()*q.y() - z()*q.z();
	double new_x = w()*q.x() + x()*q.w() + y()*q.z() - z()*q.y();
	double new_y = w()*q.y() - x()*q.z() + y()*q.w() + z()*q.x();
	double new_z = w()*q.z() + x()*q.y() - y()*q.x() + z()*q.w();
	return Quaternion(new_w, new_x, new_y, new_z);
}

Quaternion Quaternion::operator+(const Quaternion &q) const {
	return Quaternion(w()+q.w(), x()+q.x(), y()+q.y(), z()+q.z());
}

Quaternion Quaternion::operator-(const Quaternion &q) const { 
	return Quaternion(w()-q.w(), x()-q.x(), y()-q.y(), z()-q.z());
}

Quaternion Quaternion::operator/(const Quaternion &q) const {
	return q*inv();
}