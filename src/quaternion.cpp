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


double Quaternion::norm() {
	return sqrt(w()*w() + x()*x() + y()*y() + z()*z());
}

Quaternion Quaternion::adjoint() {
	return Quaternion(w(), x(), y(),z());
}

Quaternion Quaternion::inv() {
	Quaternion qTemp = adjoint();
	return qTemp/(norm()*norm());
}

bool Quaternion::isApprox(Quaternion other, double atol, double rtol) {
	return 	utils::isApprox(w(), other.w(), atol=atol, rtol=rtol) and
			utils::isApprox(x(), other.x(), atol=atol, rtol=rtol) and
			utils::isApprox(y(), other.y(), atol=atol, rtol=rtol) and
			utils::isApprox(z(), other.z(), atol=atol, rtol=rtol);
}

/**
 * Quaternion number operator overloads
 */
Quaternion operator+(Quaternion &q, double a) {return Quaternion(q.w()+a, q.x(), q.y(), q.z());}
Quaternion operator+(double a, Quaternion &q) {return Quaternion(q.w()+a, q.x(), q.y(), q.z());}

Quaternion operator*(const Quaternion &q, const double a) {return Quaternion(q.w()*a, q.x()*a, q.y()*a, q.z()*a);}
Quaternion operator*(const double a, const Quaternion &q) {return Quaternion(q.w()*a, q.x()*a, q.y()*a, q.z()*a);}

Quaternion operator/(Quaternion &q, double a) {return Quaternion(q.w()/a, q.x()/a, q.y()/a, q.z()/a);}
Quaternion operator/(double a, Quaternion &q) {Quaternion qTemp = q.inv(); return a*qTemp;}
/**
 * Quaternion Quaternion operator overloads
 */
Quaternion operator*(Quaternion &q1, Quaternion &q2) {
	double new_w = q1.w()*q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z();
	double new_x = q1.w()*q2.x() + q1.x()*q2.w() + q1.y()*q2.z() - q1.z()*q2.y();
	double new_y = q1.w()*q2.y() - q1.x()*q2.z() + q1.y()*q2.w() + q1.z()*q2.x();
	double new_z = q1.w()*q2.z() + q1.x()*q2.y() - q1.y()*q2.x() + q1.z()*q2.w();
	return Quaternion(new_w, new_x, new_y, new_z);
}

Quaternion operator+(Quaternion &q1, Quaternion &q2) {
	return Quaternion(q1.w()+q2.w(), q1.x()+q2.x(), q1.y()+q2.y(), q1.z()+q2.z());
}

Quaternion operator-(Quaternion &q1, Quaternion &q2) {
	return Quaternion(q1.w()-q2.w(), q1.x()-q2.x(), q1.y()-q2.y(), q1.z()-q2.z());
}

Quaternion operator/(Quaternion &q1, Quaternion &q2) {
	Quaternion qTemp = q2.inv();
	Quaternion out = q1*qTemp;
	return out;
}