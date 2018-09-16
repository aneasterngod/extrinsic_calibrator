/*
 * Preintegrator2.h
 *
 *  Created on: May 19, 2016
 *      Author: dongshin
 */

#ifndef COMMON_IMU_PREINTEGRATOR2_H_
#define COMMON_IMU_PREINTEGRATOR2_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <sophus/se3.hpp>
#include <iostream>
#include <string>

#include "RightJacobian.h"

using namespace std;

class Preintegrator2
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
	int id;
	Eigen::Vector3d given_bias_Acc;   // Acceleration and angular rate bias values used during preintegration
	Eigen::Vector3d given_bias_Omega; // Acceleration and angular rate bias values used during preintegration
	Eigen::Vector3d TIIC;			  // Camera-IMU displacement used during preintegration
	Eigen::Vector3d acc_noise_eta;
	Eigen::Vector3d gyro_noise_eta;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Eigen::Matrix3d DRij_w_bias;
	Eigen::Vector3d DVTij_w_bias;
	Eigen::Matrix3d DVRij_w_bias;
	Eigen::Vector3d DPTij_w_bias;
	Eigen::Matrix3d DPRij_w_bias;
	double DTij;
	int count;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Eigen::Matrix3d DRij_w_bias_DbiasGyro;
	Eigen::Matrix3d DPTij_w_bias_DbiasAcc;
	Eigen::Matrix3d DPTij_w_bias_DbiasGyro;
	Eigen::Matrix3d DPRij_w_bias_w_TIIC_DbiasGyro;
	Eigen::Matrix3d DVTij_w_bias_DbiasAcc;
	Eigen::Matrix3d DVTij_w_bias_DbiasGyro;
	Eigen::Matrix3d DVRij_w_bias_w_TIIC_DbiasGyro;

	// NOISE
	Eigen::Matrix3d Rjminus1j_inv;
	Eigen::Matrix3d Jrd;
	Eigen::Matrix<double, 9, 9> A;
	Eigen::Matrix<double, 9, 6> B;
	Eigen::Matrix<double, 9, 9> Sigma_ij;
	Eigen::Matrix<double, 9, 9> Sigma_ij_inv_sqrt;
	Eigen::Matrix<double, 6, 6> Sigma_noise;

	Eigen::Matrix<double, 9, 9> AConv;
	Eigen::Matrix<double, 9, 6> BConv;
	Eigen::Matrix<double, 9, 9> Sigma_ijConv;
	Eigen::Matrix<double, 9, 9> Sigma_ij_inv_sqrtConv;

	//	Eigen::Vector3d Rij_noise;
	//	Eigen::Vector3d VTij_noise;
	//	Eigen::Matrix3d VRij_noise;
	//	Eigen::Vector3d PTij_noise;
	//	Eigen::Matrix3d PRij_noise;

	// Connection

	Eigen::Vector3d prev_omega;
	Eigen::Vector3d curr_omega;
	Eigen::Vector3d prev_acc;
	Eigen::Vector3d curr_acc;
	Eigen::Vector3d prev_omega_w_bias;
	Eigen::Vector3d curr_omega_w_bias;
	Eigen::Vector3d prev_acc_w_bias;
	Eigen::Vector3d curr_acc_w_bias;
	Eigen::Vector3d prev_alpha_w_bias;
	Eigen::Vector3d curr_alpha_w_bias;

  private:
	bool firsttimeconnected;

  public:
	Preintegrator2();
	Preintegrator2(Preintegrator2 *preint);
	Preintegrator2(int i);
	virtual ~Preintegrator2();
	void integrateMeasurement(const Eigen::Vector3d &measuredAcc, const Eigen::Vector3d &measuredOmega, double deltaT);
	void computeISQRT(bool initialized);
	void setNoise(const Eigen::Vector3d &accnoise, const Eigen::Vector3d &gyronoise);
	void setTIIC(const Eigen::Vector3d &t);
	void setBIAS(const Eigen::Vector3d &biasgyro, const Eigen::Vector3d &biasacc);
	void setBIAS(const double *biasgyro, const double *biasacc);
	void reset();
	void setConnected(bool c);
	void print();
	string reportShort();
	void printShort();
	Preintegrator2 &operator=(const Preintegrator2 &rhs);
};

#endif /* COMMON_IMU_PREINTEGRATOR2_H_ */
