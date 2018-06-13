/*
 * RightJacobian.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: dongshin
 */

#include "RightJacobian.h"
RJ::RJ() {

}
RJ::~RJ() {

}

Eigen::Matrix<double, 3, 3> RJ::Jr(const Eigen::Matrix<double, 3, 1>& x) {
	// x is the axis-angle representation (exponential coordinates) for a rotation
	double normx = x.norm();
	Eigen::Matrix<double, 3, 3> jr;
	if (normx < 10e-8) {
		jr = Eigen::Matrix<double, 3, 3>::Identity();
	} else {
		const Eigen::Matrix<double, 3, 3> X = Sophus::SO3d::hat(x);
		jr = Eigen::Matrix<double, 3, 3>::Identity() - ((1 - cos(normx)) / (normx * normx)) * X
				+ ((normx - sin(normx)) / (normx * normx * normx)) * X * X; // right Jacobian
	}
	return jr;
}

Eigen::Matrix<double, 3, 3> RJ::Jrinv(const Eigen::Matrix<double, 3, 1>& x) {
	// x is the axis-angle representation (exponential coordinates) for a rotation
	double normx = x.norm();
	Eigen::Matrix<double, 3, 3> jrinv;

	if (normx < 10e-8) {
		jrinv = Eigen::Matrix<double, 3, 3>::Identity();
	} else {
		const Eigen::Matrix<double, 3, 3> X = Sophus::SO3d::hat(x); // element of Lie algebra so(3): X = x^
		jrinv = Eigen::Matrix<double, 3, 3>::Identity() + 0.5 * X
				+ (1 / (normx * normx) - (1 + cos(normx)) / (2 * normx * sin(normx))) * X * X;
	}
	return jrinv;
}

