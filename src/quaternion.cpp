#include "quaternion.h"

Quaternion::Quaternion() : w_{0}, x_{0}, y_{0}, z_{0} {}
Quaternion::Quaternion(double w, double x, double y, double z) : w_{w}, x_{x}, y_{y}, z_{z} {}

Quaternion::Quaternion(const Eigen::Vector3d &p) {
	w_ = 0.0;
	x_ = p(0)/p.norm();
	y_ = p(1)/p.norm();
	z_ = p(2)/p.norm();
}

Quaternion::Quaternion(const Eigen::Vector3d &axis, const double angle) {
	double s2 = sin(angle/2);
	w_ = cos(angle/2);
	x_ = s2*axis(0);
	y_ = s2*axis(1);
	z_ = s2*axis(2);
}

Quaternion::Quaternion(const double roll, const double pitch, const double yaw) {
	double cr = cos(roll*0.5);
	double sr = sin(roll*0.5);
	double cp = cos(pitch*0.5);
	double sp = sin(pitch*0.5);
	double cy = cos(yaw*0.5);
	double sy = sin(yaw*0.5);
	w_ = cr*cp*cy - sr*sp*sy ;
	x_ = sr*cp*cy + cr*sp*sy ;
	y_ = cr*sp*cy - sr*cp*sy ;
	z_ = cr*cp*sy + sr*sp*cy ;
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
	return Quaternion(w(), -x(), -y(),-z());
}

double Quaternion::norm() const {
	return sqrt(w()*w() + x()*x() + y()*y() + z()*z());
}

bool Quaternion::isRotation(double atol) const {
	return utils::isApprox(norm(), 1.0, atol);
}

bool Quaternion::isApprox(const Quaternion other, double atol, double rtol) const {
	return 	utils::isApprox(w(), other.w(), atol=atol, rtol=rtol) and
			utils::isApprox(x(), other.x(), atol=atol, rtol=rtol) and
			utils::isApprox(y(), other.y(), atol=atol, rtol=rtol) and
			utils::isApprox(z(), other.z(), atol=atol, rtol=rtol);
}

/* Rotation of a 3D vector */
Eigen::Vector3d Quaternion::operator*(const Eigen::Vector3d &vector) const{
	Quaternion vector_quaternion(vector);
	vector_quaternion = this->operator*(vector_quaternion)*adjoint();
	return {vector_quaternion.x(), vector_quaternion.y(), vector_quaternion.z()};
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

bool Quaternion::operator==(const Quaternion &q) const {
	return isApprox(q);
}

Eigen::Matrix3d Quaternion::toRotationMatrix() {
	double s = 1/(norm()*norm());
	double r = w();
	double i = x();
	double j = y();
	double k = z();

	Eigen::Matrix3d out;
	out << 1-2*s*(j*j+k*k), 2*s*(i*j-k*r)   , 2*s*(i*k+j*r)    ,
		   2*s*(i*j+k*r)  , 1-2*s*(i*i+k*k) , 2*s*(j*k-i*r)    ,
		   2*s*(i*k-j*r)  , 2*s*(j*k+i*r)   , 1-2*s*(i*i+j*j);
	return out;
}

std::pair<Eigen::Vector3d, double> Quaternion::toAxisAngle() {
	// Extract the axis from the quaternion and normalize it
	Eigen::Vector3d axis{x(), y(), z()};
	axis /= axis.norm();
	double angle = 2*atan2(axis.norm(), w());
	return std::pair<Eigen::Vector3d, double>(axis, angle);
}