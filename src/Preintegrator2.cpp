/*
 * Preintegrator2.cpp
 *
 *  Created on: May 19, 2016
 *      Author: dongshin
 */

#include "Preintegrator2.h"

Preintegrator2::Preintegrator2() {
	firsttimeconnected = false;
	count = 0;
	id = 0;
	DTij = 0;
	reset();
}

Preintegrator2::Preintegrator2(Preintegrator2* preint){
	id = preint->id;
	given_bias_Acc = preint->given_bias_Acc;
	given_bias_Omega =	preint->given_bias_Omega;
	TIIC = preint->TIIC;
	acc_noise_eta = preint->acc_noise_eta;
	gyro_noise_eta = preint->gyro_noise_eta;
	DRij_w_bias = preint->DRij_w_bias;
	DVTij_w_bias = preint->DVTij_w_bias;
	DVRij_w_bias = preint->DVRij_w_bias;
	DPTij_w_bias = preint->DPTij_w_bias;
	DPRij_w_bias = preint->DPRij_w_bias;
	DTij=preint->DTij;
	count=preint->count;
	DRij_w_bias_DbiasGyro = preint->DRij_w_bias_DbiasGyro;
	DPTij_w_bias_DbiasAcc = preint->DPTij_w_bias_DbiasAcc;
	DPTij_w_bias_DbiasGyro = preint->DPTij_w_bias_DbiasGyro;
	DPRij_w_bias_w_TIIC_DbiasGyro = preint->DPRij_w_bias_w_TIIC_DbiasGyro;
	DVTij_w_bias_DbiasAcc = preint->DVTij_w_bias_DbiasAcc;
	DVTij_w_bias_DbiasGyro = preint->DVTij_w_bias_DbiasGyro;
	DVRij_w_bias_w_TIIC_DbiasGyro = preint->DVRij_w_bias_w_TIIC_DbiasGyro;

	// NOISE
	Rjminus1j_inv = preint->Rjminus1j_inv;
	Jrd = preint->Jrd;
	A = preint->A;
	B = preint->B;
	Sigma_ij = preint->Sigma_ij;
	Sigma_ij_inv_sqrt = preint->Sigma_ij_inv_sqrt;
	Sigma_noise = preint->Sigma_noise;


	AConv = preint->AConv;
	BConv = preint->BConv;
	Sigma_ijConv = preint->Sigma_ijConv;
	Sigma_ij_inv_sqrtConv = preint->Sigma_ij_inv_sqrtConv;

	prev_omega = preint->prev_omega;
	curr_omega = preint->curr_omega;
	prev_acc = preint->prev_acc;
	curr_acc = preint->curr_acc;
	prev_omega_w_bias = preint->prev_omega_w_bias;
	curr_omega_w_bias = preint->curr_omega_w_bias;
	prev_acc_w_bias = preint->prev_acc_w_bias;
	curr_acc_w_bias = preint->curr_acc_w_bias;
	prev_alpha_w_bias = preint->prev_alpha_w_bias;
	curr_alpha_w_bias = preint->prev_alpha_w_bias;
	firsttimeconnected = preint->firsttimeconnected;
}

Preintegrator2::Preintegrator2(int i) {
	firsttimeconnected = false;
	count = 0;
	id = i;
	DTij = 0;
	reset();
}

Preintegrator2::~Preintegrator2() {
}

void Preintegrator2::integrateMeasurement(const Eigen::Vector3d& measuredAcc, const Eigen::Vector3d& measuredOmega, double deltaT) {
	if (firsttimeconnected) {
		curr_omega = measuredOmega;
		curr_acc = measuredAcc;
		curr_omega_w_bias = curr_omega - given_bias_Omega;
		curr_acc_w_bias = curr_acc - given_bias_Acc;
		firsttimeconnected = false;
	} else {
		///////////////////////////////////////////////////////
		prev_omega = curr_omega;
		prev_acc = curr_acc;
		prev_omega_w_bias = curr_omega_w_bias;
		prev_acc_w_bias = curr_acc_w_bias;
		prev_alpha_w_bias = curr_alpha_w_bias;
		///////////////////////////////////////////////////////
		curr_omega = measuredOmega;
		curr_acc = measuredAcc;
		curr_omega_w_bias = curr_omega - given_bias_Omega;
		curr_acc_w_bias = curr_acc - given_bias_Acc;
	}

	double deltaT2 = deltaT * deltaT;
	curr_alpha_w_bias = (curr_omega_w_bias - prev_omega_w_bias) / deltaT;
	///////////////////////////////////////////////////////////////////////////////////
	// Common variables
	///////////////////////////////////////////////////////////////////////////////////
	Eigen::Matrix3d bar_alphaX = Sophus::SO3d::hat(curr_alpha_w_bias);
	Eigen::Matrix3d bar_omegaX = Sophus::SO3d::hat(curr_omega_w_bias);
	Eigen::Matrix3d bar_accX = Sophus::SO3d::hat(curr_acc_w_bias);
	Eigen::Matrix3d bar_omegaX_square = bar_omegaX * bar_omegaX;
	Eigen::Matrix3d DeltaRijminus1 = DRij_w_bias;

	Eigen::Vector3d bar_at = bar_alphaX * TIIC;
	Eigen::Vector3d bar_an = bar_omegaX_square * TIIC;
	Eigen::Matrix3d bar_at_plus_bar_anX = Sophus::SO3d::hat(bar_at + bar_an);
	Eigen::Matrix3d tiicX = Sophus::SO3d::hat(TIIC);
	Eigen::Vector3d vt = bar_omegaX * TIIC;
	Eigen::Matrix3d vtX = Sophus::SO3d::hat(vt);
	Eigen::Matrix3d tiicskew = tiicX * 1.0 / deltaT + vtX + bar_omegaX * tiicX;

	///////////////////////////////////////////////////////////////////////////////////
	// Position integration
	///////////////////////////////////////////////////////////////////////////////////
	//	DPTij_w_bias += DRij_w_bias * curr_acc_w_bias * deltaT2;
	//	DPTij_w_bias += DVTij_w_bias * deltaT;
	DPTij_w_bias += DVTij_w_bias * deltaT + 0.5 * DRij_w_bias * curr_acc_w_bias * deltaT2;
	//	DPRij_w_bias += DRij_w_bias * (bar_alphaX + bar_omegaX_square) * deltaT2;
	//	DPRij_w_bias += DVRij_w_bias * deltaT;
	DPRij_w_bias += DVRij_w_bias * deltaT + 0.5 * DRij_w_bias * (bar_alphaX + bar_omegaX_square) * deltaT2;
//	PRij_noise += DeltaRijminus1 * deltaRijminus1X * (bar_alphaX + bar_omegaX_square) * deltaT2
//			- DeltaRijminus1 * etaskew * deltaT2 - DeltaRijminus1 * deltaRijminus1X * etaskew * deltaT2;
	DPTij_w_bias_DbiasAcc = DPTij_w_bias_DbiasAcc + DVTij_w_bias_DbiasAcc * deltaT - 0.5 * DRij_w_bias * deltaT2;
	DPTij_w_bias_DbiasGyro = DPTij_w_bias_DbiasGyro + DVTij_w_bias_DbiasGyro * deltaT - 0.5 * DRij_w_bias * bar_accX * DRij_w_bias_DbiasGyro * deltaT2;
	DPRij_w_bias_w_TIIC_DbiasGyro = DPRij_w_bias_w_TIIC_DbiasGyro + DVRij_w_bias_w_TIIC_DbiasGyro * deltaT - 0.5 * DRij_w_bias * bar_at_plus_bar_anX * DRij_w_bias_DbiasGyro * deltaT2
			+ 0.5 * DRij_w_bias * (tiicskew) * deltaT2;

	///////////////////////////////////////////////////////////////////////////////////
	// Velocity integration
	///////////////////////////////////////////////////////////////////////////////////
	DVTij_w_bias += DeltaRijminus1 * curr_acc_w_bias * deltaT;
	DVRij_w_bias += DeltaRijminus1 * (bar_alphaX + bar_omegaX_square) * deltaT;
//	cout << "DT: " << deltaT << endl;
//	cout << "curr_alpha_w_bias" << endl;
//	cout << curr_alpha_w_bias << endl;
//	cout << "BAX" << endl;
//	cout << bar_alphaX << endl;
//	cout << "BAOMEGA" << endl;
//	cout << bar_omegaX_square << endl;
//	cout << "DeltaRijminus1" << endl;
//	cout << DeltaRijminus1 << endl;
//	cout << "DVRij_w_bias" << endl;
//	cout << DVRij_w_bias << endl;


//	VTij_noise = VTij_noise - DeltaRijminus1 * bar_accX * deltaRijminus1 * deltaT - DeltaRijminus1 * acc_noise_eta * deltaT
//			- DeltaRijminus1 * deltaRijminus1X * acc_noise_eta * deltaT;
//	VRij_noise = VRij_noise + DeltaRijminus1 * deltaRijminus1X * (bar_alphaX + bar_omegaX_square) * deltaT
//			- DeltaRijminus1 * etaskew * deltaT - DeltaRijminus1 * deltaRijminus1X * etaskew * deltaT;
	DVTij_w_bias_DbiasAcc = DVTij_w_bias_DbiasAcc - DeltaRijminus1 * deltaT;
	DVTij_w_bias_DbiasGyro = DVTij_w_bias_DbiasGyro - DeltaRijminus1 * bar_accX * DRij_w_bias_DbiasGyro * deltaT;
	DVRij_w_bias_w_TIIC_DbiasGyro = DVRij_w_bias_w_TIIC_DbiasGyro - DRij_w_bias * bar_at_plus_bar_anX * DRij_w_bias_DbiasGyro * deltaT + DRij_w_bias * tiicskew * deltaT;

	///////////////////////////////////////////////////////////////////////////////////
	// Compute measurement covariance
	///////////////////////////////////////////////////////////////////////////////////
	// Noise p, v, R, (bias, C, T)
	// state evolves, but bias C T is just random walk process. No systemaTIIC noise on them.
	// so only noise we have to concern is p v R, the rest of them are just zero mean isormophic gaussian.
	Eigen::Matrix3d Zero3x3 = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
//	Eigen::Matrix3d DeltaRijminus1_aXplus_tanacc_normalaccX = -DeltaRijminus1 * (bar_accX + bar_at_plus_bar_anX) * deltaT;
//	Eigen::Matrix3d DeltaRijminus1_aXplus_tanacc_normalaccX2 = DeltaRijminus1_aXplus_tanacc_normalaccX * deltaT;
//	Eigen::Matrix3d DeltaRijminus1_deltaT = -DeltaRijminus1 * deltaT;
//	Eigen::Matrix3d DeltaRijminus1_deltaT2 = DeltaRijminus1_deltaT * deltaT;
//	Eigen::Matrix3d DeltaRijminus1_TIICX_vX_omegaX_TIICX_dt = DeltaRijminus1 * (tiicX * 1.0 / deltaT + vtX + bar_omegaX * tiicX)
//			* deltaT;
//	Eigen::Matrix3d DeltaRijminus1_TIICX_vX_omegaX_TIICX_dt2 = DeltaRijminus1_TIICX_vX_omegaX_TIICX_dt * deltaT;

	///////////////////////////////////////////////////////////////////////////////////
	// ROTATION integration
	///////////////////////////////////////////////////////////////////////////////////
	const Eigen::Vector3d curr_omega_w_bias_times_dt = curr_omega_w_bias * deltaT;
	// add up to make DeltaRij
	Sophus::SO3d Exp_curr_omega_w_bias_times_dt = Sophus::SO3d::exp(curr_omega_w_bias_times_dt);
	// Compute Jacobians for bias update
	Eigen::Matrix3d Jr = RJ::Jr(curr_omega_w_bias_times_dt);
//	Eigen::Matrix3d Jr_inv = RJ::Jrinv(curr_omega_w_bias_times_dt);
	Jrd = Jr * deltaT;
	Rjminus1j_inv = Exp_curr_omega_w_bias_times_dt.inverse().matrix();
	Eigen::Matrix3d Rjminus1j = Exp_curr_omega_w_bias_times_dt.matrix();
	DRij_w_bias = DRij_w_bias * Rjminus1j;
//	Rij_noise = Rjminus1j * Rij_noise - Jr * gyro_noise_eta * deltaT;
	DRij_w_bias_DbiasGyro = -Rjminus1j_inv * DRij_w_bias_DbiasGyro - Jrd;

	A << Rjminus1j_inv, Zero3x3, Zero3x3,
		-DeltaRijminus1 * (bar_accX + bar_at_plus_bar_anX) * deltaT, I3x3, Zero3x3,
		-0.5 * DeltaRijminus1 * (bar_accX + bar_at_plus_bar_anX) * deltaT2, I3x3 * deltaT, Zero3x3;
	B << Jrd, Zero3x3,
		 -DeltaRijminus1 * tiicskew * deltaT, DeltaRijminus1 * deltaT,
		 -0.5 * DeltaRijminus1 * tiicskew * deltaT2, 0.5 * DeltaRijminus1 * deltaT2;

	Sigma_ij = A * Sigma_ij * A.transpose() + B * Sigma_noise * B.transpose();

	AConv << Rjminus1j_inv, Zero3x3, Zero3x3,
			-DeltaRijminus1 * deltaT, I3x3, Zero3x3,
			-0.5 * DeltaRijminus1 * deltaT2, I3x3 * deltaT, Zero3x3;
	BConv << Jrd, Zero3x3, -DeltaRijminus1 * deltaT, DeltaRijminus1 * deltaT, -0.5 * DeltaRijminus1 * deltaT2, 0.5 * DeltaRijminus1 * deltaT2;
	Sigma_ijConv = AConv * Sigma_ijConv * AConv.transpose() + BConv * Sigma_noise * BConv.transpose();

	DTij += deltaT;
	count++;

}

void Preintegrator2::computeISQRT(bool initialized) {
	// 만약 inverse에 음수가 있다면... 이것은 이상한 value가 된다.
//	cout << Sigma_ij << endl;

	if(initialized){
//		Sigma_ij_inv_sqrt = Sigma_ij.inverse().sqrt();
//		Sigma_ij_inv_sqrtConv = Sigma_ijConv.inverse().sqrt();
		Sigma_ij_inv_sqrt = Sigma_ij.sqrt().inverse();
		Sigma_ij_inv_sqrtConv = Sigma_ijConv.sqrt().inverse();

	}
	else{
		Sigma_ij_inv_sqrt << 1, 0, 0, 0, 0, 0, 0, 0, 0,
				                    0, 1, 0, 0, 0, 0, 0, 0, 0,
							  0, 0, 1, 0, 0, 0, 0, 0, 0,
							  0, 0, 0, 1, 0, 0, 0, 0, 0,
							  0, 0, 0, 0, 1, 0, 0, 0, 0,
							  0, 0, 0, 0, 0, 1, 0, 0, 0,
							  0, 0, 0, 0, 0, 0, 1, 0, 0,
							  0, 0, 0, 0, 0, 0, 0, 1, 0,
							  0, 0, 0, 0, 0, 0, 0, 0, 1;
		Sigma_ij_inv_sqrtConv << 1, 0, 0, 0, 0, 0, 0, 0, 0,
				                    0, 1, 0, 0, 0, 0, 0, 0, 0,
							  0, 0, 1, 0, 0, 0, 0, 0, 0,
							  0, 0, 0, 1, 0, 0, 0, 0, 0,
							  0, 0, 0, 0, 1, 0, 0, 0, 0,
							  0, 0, 0, 0, 0, 1, 0, 0, 0,
							  0, 0, 0, 0, 0, 0, 1, 0, 0,
							  0, 0, 0, 0, 0, 0, 0, 1, 0,
							  0, 0, 0, 0, 0, 0, 0, 0, 1;
	}
	cout << Sigma_ij << endl;
	cout << "======================" << endl;
	cout << Sigma_ij.sqrt() << endl;
	cout << "======================" << endl;
	cout << Sigma_ij_inv_sqrt << endl;
}

string Preintegrator2::reportShort(){
	std::stringstream buffer;
	buffer << "DT: " << DTij << endl;
	buffer << "DR" << endl;
	buffer << DRij_w_bias << endl;
	buffer << "DV: " << DVTij_w_bias.transpose() << endl;
	buffer << "DVR: " << endl;
	buffer << DVRij_w_bias << endl;
	buffer << "DP: " << DPTij_w_bias.transpose() << endl;
	buffer << "DPR: " << endl;
	buffer << DPRij_w_bias << endl;
	buffer << "Count: " << count << endl;
	return buffer.str();
}
void Preintegrator2::printShort(){
	cout << "DT: " << DTij << endl;
	cout << "DR" << endl;
	cout << DRij_w_bias << endl;
	cout << "DV: " << DVTij_w_bias.transpose() << endl;
	cout << "DVR: " << endl;
	cout << DVRij_w_bias << endl;
	cout << "DP: " << DPTij_w_bias.transpose() << endl;
	cout << "DPR: " << endl;
	cout << DPRij_w_bias << endl;
	cout << "Count: " << count << endl;
}

void Preintegrator2::setNoise(const Eigen::Vector3d& accnoise, const Eigen::Vector3d& gyronoise) {
	acc_noise_eta = accnoise;
	gyro_noise_eta = gyronoise;
	Sigma_noise << gyro_noise_eta(0), 0, 0, 0, 0, 0, 0, gyro_noise_eta(1), 0, 0, 0, 0, 0, 0, gyro_noise_eta(2), 0, 0, 0, 0, 0, 0, acc_noise_eta(0), 0, 0, 0, 0, 0, 0, acc_noise_eta(1), 0, 0, 0, 0, 0, 0, acc_noise_eta(
			2);
}

void Preintegrator2::setTIIC(const Eigen::Vector3d& t) {
	TIIC = t;
}

void Preintegrator2::reset() {
	given_bias_Acc << 0, 0, 0;
	given_bias_Omega << 0, 0, 0;
	TIIC << 0, 0, 0;
	acc_noise_eta << 0, 0, 0;
	gyro_noise_eta << 0, 0, 0;
	DRij_w_bias << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	DVTij_w_bias << 0, 0, 0;
	DVRij_w_bias << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	DPTij_w_bias << 0, 0, 0;
	DPRij_w_bias << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	DTij = 0;
	count = 0;

	DRij_w_bias_DbiasGyro << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	DPTij_w_bias_DbiasAcc << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	DPTij_w_bias_DbiasGyro << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	DPRij_w_bias_w_TIIC_DbiasGyro << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	DVTij_w_bias_DbiasAcc << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	DVTij_w_bias_DbiasGyro << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	DVRij_w_bias_w_TIIC_DbiasGyro << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	// NOISE
	A << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	Sigma_ij << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	Sigma_ij_inv_sqrt << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	Sigma_noise << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
//	Rij_noise << 0, 0, 0;
//	VTij_noise << 0, 0, 0;
//	VRij_noise << 0, 0, 0, 0, 0, 0, 0, 0, 0;
//	PTij_noise << 0, 0, 0;
//	PRij_noise << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	prev_omega << 0, 0, 0;
	curr_omega << 0, 0, 0;
	prev_acc << 0, 0, 0;
	curr_acc << 0, 0, 0;
	prev_omega_w_bias << 0, 0, 0;
	curr_omega_w_bias << 0, 0, 0;
	prev_acc_w_bias << 0, 0, 0;
	curr_acc_w_bias << 0, 0, 0;
	prev_alpha_w_bias << 0, 0, 0;
	curr_alpha_w_bias << 0, 0, 0;
}

void Preintegrator2::setConnected(bool c) {
	firsttimeconnected = c;
}
void Preintegrator2::setBIAS(const Eigen::Vector3d& biasgyro, const Eigen::Vector3d& biasacc) {
	given_bias_Acc = biasacc;
	given_bias_Omega = biasgyro;
}
void Preintegrator2::setBIAS(const double* biasgyro, const double* biasacc) {
	given_bias_Acc << biasacc[0], biasacc[1], biasacc[2];
	given_bias_Omega << biasgyro[0], biasgyro[1], biasgyro[2];
}
void Preintegrator2::print() {
	cout << "Sigma_noise" << endl;
	cout << Sigma_noise << endl;
	cout << "Sigma_ij" << endl;
	cout << Sigma_ij << endl;
	cout << "Sigma_ij_inv_sqrt" << endl;
	cout << Sigma_ij_inv_sqrt << endl;
	cout << "DT: " << DTij << endl;
	cout << "DR" << endl;
	cout << DRij_w_bias << endl;
	cout << "DV: " << DVTij_w_bias.transpose() << endl;
	cout << "DVR: " << endl;
	cout << DVRij_w_bias << endl;
	cout << "DP: " << DPTij_w_bias.transpose() << endl;
	cout << "DPR: " << endl;
	cout << DPRij_w_bias << endl;
	cout << "Count: " << count << endl;
}
Preintegrator2& Preintegrator2::operator=(const Preintegrator2 &rhs) {
	id = rhs.id;
	given_bias_Acc = rhs.given_bias_Acc; // Acceleration and angular rate bias values used during preintegration
	given_bias_Omega = rhs.given_bias_Omega; // Acceleration and angular rate bias values used during preintegration
	TIIC = rhs.TIIC; // Camera-IMU displacement used during preintegration
	acc_noise_eta = rhs.acc_noise_eta;
	gyro_noise_eta = rhs.gyro_noise_eta;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DRij_w_bias = rhs.DRij_w_bias;
	DVTij_w_bias = rhs.DVTij_w_bias;
	DVRij_w_bias = rhs.DVRij_w_bias;
	DPTij_w_bias = rhs.DPTij_w_bias;
	DPRij_w_bias = rhs.DPRij_w_bias;
	DTij = rhs.DTij;
	count = rhs.count;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DRij_w_bias_DbiasGyro = rhs.DRij_w_bias_DbiasGyro;
	DPTij_w_bias_DbiasAcc = rhs.DPTij_w_bias_DbiasAcc;
	DPTij_w_bias_DbiasGyro = rhs.DPTij_w_bias_DbiasGyro;
	DPRij_w_bias_w_TIIC_DbiasGyro = rhs.DPRij_w_bias_w_TIIC_DbiasGyro;
	DVTij_w_bias_DbiasAcc = rhs.DVTij_w_bias_DbiasAcc;
	DVTij_w_bias_DbiasGyro = rhs.DVTij_w_bias_DbiasGyro;
	DVRij_w_bias_w_TIIC_DbiasGyro = rhs.DVRij_w_bias_w_TIIC_DbiasGyro;

	// NOISE
	A = rhs.A;
	B = rhs.B;
	Sigma_ij = rhs.Sigma_ij;
	Sigma_ij_inv_sqrt = rhs.Sigma_ij_inv_sqrt;
	Sigma_noise = rhs.Sigma_noise;
	//	Eigen::Vector3d Rij_noise;
	//	Eigen::Vector3d VTij_noise;
	//	Eigen::Matrix3d VRij_noise;
	//	Eigen::Vector3d PTij_noise;
	//	Eigen::Matrix3d PRij_noise;

	// Connection

	prev_omega = rhs.prev_omega;
	curr_omega = rhs.curr_omega;
	prev_acc = rhs.prev_acc;
	curr_acc = rhs.curr_acc;
	prev_omega_w_bias = rhs.prev_omega_w_bias;
	curr_omega_w_bias = rhs.curr_omega_w_bias;
	prev_acc_w_bias = rhs.prev_acc_w_bias;
	curr_acc_w_bias = rhs.curr_acc_w_bias;
	prev_alpha_w_bias = rhs.prev_alpha_w_bias;
	curr_alpha_w_bias = rhs.curr_alpha_w_bias;
	return *this;
}
