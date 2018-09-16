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

using ceres::CostFunction;
using ceres::NumericDiffCostFunction;
using ceres::Problem;
using ceres::SizedCostFunction;
using ceres::Solve;
using ceres::Solver;

using namespace std;

class ceres_BA
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
	Eigen::Vector3d m_ipt;
	Eigen::Matrix3d m_K;

  public:
	ceres_BA(const Eigen::Vector3d &ipt, const Eigen::Matrix3d &K);
	virtual ~ceres_BA();
	bool operator()(const double *const pose, const double *const landmarks, double *residual) const;
};

#endif /* LOCALIZER_SRC_BA_CERES_BA_H_ */
