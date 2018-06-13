/*
 * RightJacobian.h
 *
 *  Created on: Apr 9, 2016
 *      Author: dongshin
 */

#ifndef COMMON_GEOMETRY_RIGHTJACOBIAN_H_
#define COMMON_GEOMETRY_RIGHTJACOBIAN_H_
#include <sophus/so3.hpp>

class RJ {
public:
	RJ();
	~RJ();
	static Eigen::Matrix<double, 3, 3> Jr(const Eigen::Matrix<double, 3, 1>& x);
	static Eigen::Matrix<double, 3, 3> Jrinv(const Eigen::Matrix<double, 3, 1>& x);

};

#endif /* COMMON_GEOMETRY_RIGHTJACOBIAN_H_ */
