/*
 * ceresBA.h
 *
 *  Created on: Jul 9, 2016
 *      Author: dongshin
 */

#ifndef LOCALIZER_SRC_BA_CERES_BA_H_
#define LOCALIZER_SRC_BA_CERES_BA_H_

#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <iostream>

using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;


class ceres_BA {
public:
	Eigen::Vector3d m_ipt;
	Eigen::Matrix3d m_K;
public:
	ceres_BA(Eigen::Vector3d& ipt, Eigen::Matrix3d& K);
	virtual ~ceres_BA();
	bool operator()(const double* const pose, const double* const landmarks, double* residual) const;
};

#endif /* LOCALIZER_SRC_BA_CERES_BA_H_ */
