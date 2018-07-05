/*
 * ceresBA.cpp
 *
 *  Created on: Jul 9, 2016
 *      Author: dongshin
 */

#include "ceres_BA.h"

ceres_BA::ceres_BA(Eigen::Vector3d& ipt, Eigen::Matrix3d& K) :
		m_ipt(ipt), m_K(K) {

}

ceres_BA::~ceres_BA() {
}

bool ceres_BA::operator()(const double* const pose, const double* const landmarks, double* residual) const {
	Eigen::Matrix<double, 6, 1> eigen_pose(pose);
	Sophus::SE3d se3_pose = Sophus::SE3d::exp(eigen_pose);
	Sophus::SE3d se3_pose_inv = se3_pose.inverse();
	Eigen::Vector3d eigen_landmarks(landmarks);

	Eigen::Vector3d eigen_locallandmark = se3_pose_inv * eigen_landmarks;
	Eigen::Vector3d predict_ipt = m_K * eigen_locallandmark;

	for (int i = 0; i < 3; i++) {
		predict_ipt(i) /= predict_ipt(2);
	}
	Eigen::Vector3d f = m_ipt - predict_ipt;

	Eigen::Matrix<double, 3, 1> r;
	r << f;
	for (int i = 0; i < 3; i++) {
		if (!isnan(r(i)))
			residual[i] = r(i);
		else {
			cout << "Given Landmark: " << eigen_landmarks.transpose() << endl;
			cout << "Given pose: " << se3_pose_inv.log().transpose() << endl;
			cout << "Computed point: " << predict_ipt.transpose() << endl;
			cout << "Measured point : " << m_ipt.transpose() << endl;
			cout << "Error redisual     : " << r.transpose() << endl;

			residual[i] = 100;
		}
	}
	return true;
}
