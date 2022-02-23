#ifndef _quaternion_h_
#define _quaternion_h_

#include <math.h>
#include <Eigen/Core>
#include "robo_utils.h"

class Quaternion
{
private:
	double w_;
	double x_;
	double y_;
	double z_;

public:
	/**
	 * Construct quaternion with the four elements w + xi + yj + zk
	 */
	Quaternion();
	Quaternion(const double w=0, double x=0, double y=0, double z=0);
	/**
	 * Construct quaternion from a 3D vector
	 */
	Quaternion(Eigen::Vector3d p = {0, 0, 0});

	~Quaternion();

	/* Getters */
	Eigen::Vector4d data() const {return Eigen::Vector4d{w_, x_, y_, z_};}
	double w() const {return w_;}
	double x() const {return x_;}
	double y() const {return y_;}
	double z() const {return z_;}

	/* Output functions */
	std::string toString() const;
	std::string toRepr() const;

	/**
	 * @brief Inverse of the quaternion
	 */
	Quaternion inv() const;
	/**
	 * @brief The norm of the quaternion
	 */
	double norm() const;
	/**
	 * The adjoint or conjugate
	 */
	Quaternion adjoint() const;

	bool isApprox(Quaternion other, double atol=0, double rtol=std::numeric_limits<double>::epsilon());

	/**
	 * Quaternion number operator overloads
	 */
	Quaternion operator+(const double a) const;
	friend Quaternion operator+(const double a, const Quaternion &q);

	Quaternion operator-(const double a) const;
	friend Quaternion operator-(const double a, const Quaternion &q);

	Quaternion operator*(const double a) const;
	friend Quaternion operator*(const double a, const Quaternion &q);

	Quaternion operator/(const double a) const;
	friend Quaternion operator/(const double a, const Quaternion &q);

	/**
	 * Quaternion Quaternion operator overloads
	 */
	Quaternion operator*(const Quaternion &q) const;
	Quaternion operator+(const Quaternion &q) const;
	Quaternion operator-(const Quaternion &q) const;
	Quaternion operator/(const Quaternion &q) const;
};

#endif //_quaternion_h_ header