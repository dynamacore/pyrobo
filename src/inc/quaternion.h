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
	Quaternion(const Eigen::Vector3d &p);
	/**
	 * Construct a quaternion from an axis angle 
	 */
	Quaternion(const Eigen::Vector3d &axis, const double angle);
	/**
	 * Construct a quaternion from euler angles XYZ
	 */
	Quaternion(const double roll, const double pitch, const double yaw);

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
	 * The adjoint or conjugate
	 */
	Quaternion adjoint() const;
	/**
	 * @brief The norm of the quaternion
	 */
	double norm() const;

	bool isRotation(double atol=1E-12) const;

	bool isApprox(const Quaternion other, double atol=0, double rtol=std::numeric_limits<double>::epsilon()) const;

	/**
	 * Quaternion vector rotation
	 */
	Eigen::Vector3d operator*(const Eigen::Vector3d &vector) const;

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
	bool operator==(const Quaternion &q) const;

	/**
	 * Methods for converting Quaternions to other rotation representations
	 */
	Eigen::Matrix3d toRotationMatrix();
	std::pair<Eigen::Vector3d, double> toAxisAngle();

};

#endif //_quaternion_h_ header