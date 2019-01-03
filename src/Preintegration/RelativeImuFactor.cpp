/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RelativeImuFactor.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include "Preintegration/RelativeImuFactor.h"

/* External or standard includes */
#include <ostream>

namespace gtsam
{

using namespace std;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RelativePreintegratedImuMeasurements::setLeverarm(const Vector3 &tic)
{
  TIC_ = tic;
}

void RelativePreintegratedImuMeasurements::setPrevOmega(const Vector3 &w)
{
  prevOmega_ = w;
}

Matrix3 RelativePreintegratedImuMeasurements::Jr(const Vector3 &x)
{
  // x is the axis-angle representation (exponential coordinates) for a rotation
  double normx = x.norm();
  Matrix3 jr;
  if (normx < 10e-8)
  {
    jr = Matrix3::Identity();
  }
  else
  {
    const Matrix3 X = skewSymmetric(x);
    jr = Matrix3::Identity() - ((1 - cos(normx)) / (normx * normx)) * X + ((normx - sin(normx)) / (normx * normx * normx)) * X * X; // right Jacobian
  }
  return jr;
}
Matrix3 RelativePreintegratedImuMeasurements::Jrinv(const Vector3 &x)
{
  // x is the axis-angle representation (exponential coordinates) for a rotation
  double normx = x.norm();
  Eigen::Matrix<double, 3, 3> jrinv;

  if (normx < 10e-8)
  {
    jrinv = Eigen::Matrix<double, 3, 3>::Identity();
  }
  else
  {
    const Eigen::Matrix<double, 3, 3> X = skewSymmetric(x); // element of Lie algebra so(3): X = x^
    jrinv = Eigen::Matrix<double, 3, 3>::Identity() + 0.5 * X + (1 / (normx * normx) - (1 + cos(normx)) / (2 * normx * sin(normx))) * X * X;
  }
  return jrinv;
}

//------------------------------------------------------------------------------
// Inner class RelativePreintegratedImuMeasurements
//------------------------------------------------------------------------------

void RelativePreintegratedImuMeasurements::print(const string &s) const
{
  PreintegrationType::print(s);
  cout << "    detlatRPij_: " << endl;
  cout << deltaRPij_ << endl;
  cout << "    detlatRVij_: " << endl;
  cout << deltaRVij_ << endl;
  cout << "    preintMeasCov \n[" << preintMeasCov_ << "]" << endl;
}

//------------------------------------------------------------------------------
bool RelativePreintegratedImuMeasurements::equals(
    const RelativePreintegratedImuMeasurements &other, double tol) const
{
  return PreintegrationType::equals(other, tol) &&
         equal_with_abs_tol(delRPdelBiasOmega_, other.delRPdelBiasOmega_, tol) &&
         equal_with_abs_tol(delRVdelBiasOmega_, other.delRVdelBiasOmega_, tol) &&
         equal_with_abs_tol(deltaRPij_, other.deltaRPij_, tol) &&
         equal_with_abs_tol(deltaRVij_, other.deltaRVij_, tol) &&
         equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);
}

//------------------------------------------------------------------------------
void RelativePreintegratedImuMeasurements::resetIntegration()
{
  deltaRPij_.setZero();
  deltaRVij_.setZero();
  delRPdelBiasOmega_.setZero();
  delRVdelBiasOmega_.setZero();
  prevOmega_.setZero();

  PreintegrationType::resetIntegration();
  preintMeasCov_.setZero();
}
//------------------------------------------------------------------------------
void RelativePreintegratedImuMeasurements::integrateMeasurement(
    const Vector3 &measuredAcc, const Vector3 &measuredOmega, double dt)
{
  // Update preintegrated measurements (also get Jacobian)
  Matrix9 A; // overall Jacobian wrt preintegrated measurements (df/dx)
  Matrix93 B, C;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Correct for bias in the sensor frame
  Vector3 acc = biasHat_.correctAccelerometer(measuredAcc);
  Vector3 omega = biasHat_.correctGyroscope(measuredOmega);

  double dt2 = dt * dt;
  Vector3 alpha = (omega - prevOmega_) / dt;
  Matrix3 skew_alpha = skewSymmetric(alpha);
  Matrix3 skew_omega = skewSymmetric(omega);
  Matrix3 skew_omega2 = skew_omega * skew_omega;
  Vector3 TIIC = TIC_;
  Vector3 aTAN = skew_alpha * TIIC;
  Vector3 aNOR = skew_omega2 * TIIC;
  Vector3 Upsilon = aTAN + aNOR;
  Matrix3 skew_Upsilon = skewSymmetric(Upsilon);
  Matrix3 skew_acc = skewSymmetric(acc);
  Matrix3 jr = Jr(omega * dt);
  Matrix3 skew_tiic = skewSymmetric(TIIC);
  Matrix3 skew_vTAN = skewSymmetric(skew_omega * TIIC);
  Matrix3 Gamma = skew_tiic * 1.0 / dt + skew_omega * skew_tiic + skew_vTAN;

  // Possibly correct for sensor pose
  Matrix3 D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega;
  // if (p().body_P_sensor)
  //   boost::tie(acc, omega) = correctMeasurementsBySensorPose(acc, omega,
  //       D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega);

  // Save current rotation for updating Jacobians
  const Rot3 oldRij = deltaXij_.attitude();

  // Do update
  deltaTij_ += dt;
  deltaXij_ = deltaXij_.update(acc, omega, dt, A, B, C); // functional

  deltaRPij_ = deltaRPij_ + deltaRVij_ * dt + (0.5 * oldRij.matrix() * (skew_alpha + skew_omega2) * dt2);
  Matrix3 tmpGamma = oldRij.matrix() * (skew_Upsilon * delRdelBiasOmega_ - Gamma);
  delRPdelBiasOmega_ = delRPdelBiasOmega_ + delRVdelBiasOmega_ * dt - 0.5 * tmpGamma * dt2;
  deltaRVij_ = deltaRVij_ + oldRij.matrix() * (skew_alpha + skew_omega2) * dt;
  delRVdelBiasOmega_ = delRVdelBiasOmega_ - tmpGamma * dt;

  // Update Jacobians
  // TODO(frank): Try same simplification as in new approach
  Matrix3 D_acc_R;
  oldRij.rotate(acc, D_acc_R);
  const Matrix3 D_acc_biasOmega = D_acc_R * delRdelBiasOmega_;

  const Vector3 integratedOmega = omega * dt;
  Matrix3 D_incrR_integratedOmega;
  const Rot3 incrR = Rot3::Expmap(integratedOmega, D_incrR_integratedOmega); // expensive !!
  const Matrix3 incrRt = incrR.transpose();
  delRdelBiasOmega_ = incrRt * delRdelBiasOmega_ - D_incrR_integratedOmega * dt;

  double dt22 = 0.5 * dt * dt;
  const Matrix3 dRij = oldRij.matrix(); // expensive
  delPdelBiasAcc_ += delVdelBiasAcc_ * dt - dt22 * dRij;
  delPdelBiasOmega_ += dt * delVdelBiasOmega_ + dt22 * D_acc_biasOmega;
  delVdelBiasAcc_ += -dRij * dt;
  delVdelBiasOmega_ += D_acc_biasOmega * dt;

  Matrix9 RA;
  Matrix96 RB;

  Matrix3 Zero3x3;
  Matrix3 I3x3;
  Zero3x3.setZero();
  I3x3.setIdentity();

  RA << incrR.matrix().inverse(), Zero3x3, Zero3x3,
      -0.5 * oldRij.matrix() * (skew_acc + skew_Upsilon) * dt2, I3x3, I3x3 * dt,
      -oldRij.matrix() * (skew_acc + skew_Upsilon) * dt, Zero3x3, I3x3;
  RB << jr * dt, Zero3x3,
      -0.5 * oldRij.matrix() * Gamma * dt2, 0.5 * oldRij.matrix() * dt2,
      -oldRij.matrix() * Gamma * dt, oldRij.matrix() * dt;
  A = RA;
  B = RB.rightCols<3>();
  C = RB.leftCols<3>();
  prevOmega_ = omega;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // first order covariance propagation:
  // as in [2] we consider a first order propagation that can be seen as a
  // prediction phase in EKF

  // propagate uncertainty
  // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
  const Matrix3 &aCov = p().accelerometerCovariance;
  const Matrix3 &wCov = p().gyroscopeCovariance;
  const Matrix3 &iCov = p().integrationCovariance;

  // (1/dt) allows to pass from continuous time noise to discrete time noise
  preintMeasCov_ = A * preintMeasCov_ * A.transpose();
  preintMeasCov_.noalias() += B * (aCov / dt) * B.transpose();
  preintMeasCov_.noalias() += C * (wCov / dt) * C.transpose();

  // NOTE(frank): (Gi*dt)*(C/dt)*(Gi'*dt), with Gi << Z_3x3, I_3x3, Z_3x3
  preintMeasCov_.block<3, 3>(3, 3).noalias() += iCov * dt;
}
//------------------------------------------------------------------------------
Vector9 RelativePreintegratedImuMeasurements::computeErrorAndJacobians(const Pose3 &pose_i, const Vector3 &vel_i, const Pose3 &pose_j, const Vector3 &vel_j,
                                                                       const Pose3 &extrinsic, const imuBias::ConstantBias &bias_i,
                                                                       OptionalJacobian<9, 6> H1, OptionalJacobian<9, 3> H2, OptionalJacobian<9, 6> H3,
                                                                       OptionalJacobian<9, 3> H4, OptionalJacobian<9, 6> H5, OptionalJacobian<9, 6> H6) const
{
  // this one should be changed
  // gravity is not estimated? 
  // I think it has to be estimated here.
  
  NavState state_i(pose_i, vel_i);
  NavState state_j(pose_j, vel_j);

  // Predict state at time j
  Matrix9 D_error_state_i, D_error_state_j;
  Vector9 error = computeError(state_i, state_j, bias_i,
                               H1 || H2 ? &D_error_state_i : 0, H3 || H4 ? &D_error_state_j : 0, H5);
  if (H1)
    *H1 << D_error_state_i.leftCols<6>();
  if (H2)
    *H2 << D_error_state_i.rightCols<3>() * state_i.R().transpose();
  if (H3)
    *H3 << D_error_state_j.leftCols<6>();
  if (H4)
    *H4 << D_error_state_j.rightCols<3>() * state_j.R().transpose();

  return error;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
void RelativePreintegratedImuMeasurements::integrateMeasurements(
    const Matrix &measuredAccs, const Matrix &measuredOmegas,
    const Matrix &dts)
{
  assert(
      measuredAccs.rows() == 3 && measuredOmegas.rows() == 3 && dts.rows() == 1);
  assert(dts.cols() >= 1);
  assert(measuredAccs.cols() == dts.cols());
  assert(measuredOmegas.cols() == dts.cols());
  size_t n = static_cast<size_t>(dts.cols());
  for (size_t j = 0; j < n; j++)
  {
    integrateMeasurement(measuredAccs.col(j), measuredOmegas.col(j), dts(0, j));
  }
}

//------------------------------------------------------------------------------
Vector9 RelativePreintegratedImuMeasurements::biasCorrectedDelta(
    const imuBias::ConstantBias &bias_i, OptionalJacobian<9, 6> H) const
{
  // Correct deltaRij, derivative is delRdelBiasOmega_
  const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
  Matrix3 D_correctedRij_bias;
  const Vector3 biasInducedOmega = delRdelBiasOmega_ * biasIncr.gyroscope();
  const Rot3 correctedRij = deltaRij().expmap(biasInducedOmega, boost::none,
                                              H ? &D_correctedRij_bias : 0);
  if (H)
    D_correctedRij_bias *= delRdelBiasOmega_;

  Vector9 xi;
  Matrix3 D_dR_correctedRij;

  NavState::dR(xi) = (Rot3::Logmap(correctedRij, H ? &D_dR_correctedRij : 0));
  NavState::dP(xi) = (deltaPij() + delPdelBiasAcc_ * biasIncr.accelerometer() + delPdelBiasOmega_ * biasIncr.gyroscope() + (deltaRPij() * TIC_ + delRPdelBiasOmega_ * biasIncr.gyroscope()));
  NavState::dV(xi) = (deltaVij() + delVdelBiasAcc_ * biasIncr.accelerometer() + delVdelBiasOmega_ * biasIncr.gyroscope() + (deltaRVij() * TIC_ + delRVdelBiasOmega_ * biasIncr.gyroscope()));

  if (H)
  {
    Matrix36 D_dR_bias, D_dP_bias, D_dV_bias;
    D_dR_bias << Z_3x3, D_dR_correctedRij * D_correctedRij_bias;
    D_dP_bias << delPdelBiasAcc_, (delPdelBiasOmega_ + delRPdelBiasOmega_);
    D_dV_bias << delVdelBiasAcc_, (delVdelBiasOmega_ + delRVdelBiasOmega_);
    (*H) << D_dR_bias, D_dP_bias, D_dV_bias;
  }
  return xi;
}

//------------------------------------------------------------------------------
// #ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
// RelativePreintegratedImuMeasurements::RelativePreintegratedImuMeasurements(
//     const imuBias::ConstantBias &biasHat, const Matrix3 &measuredAccCovariance,
//     const Matrix3 &measuredOmegaCovariance,
//     const Matrix3 &integrationErrorCovariance, bool use2ndOrderIntegration)
// {
//   if (!use2ndOrderIntegration)
//     throw("RelativePreintegratedImuMeasurements no longer supports first-order integration: it incorrectly compensated for gravity");
//   biasHat_ = biasHat;
//   boost::shared_ptr<Params> p = Params::MakeSharedD();
//   p->gyroscopeCovariance = measuredOmegaCovariance;
//   p->accelerometerCovariance = measuredAccCovariance;
//   p->integrationCovariance = integrationErrorCovariance;
//   p_ = p;
//   resetIntegration();
// }

// void RelativePreintegratedImuMeasurements::integrateMeasurement(
//     const Vector3 &measuredAcc, const Vector3 &measuredOmega, double deltaT,
//     boost::optional<Pose3> body_P_sensor)
// {
//   // modify parameters to accommodate deprecated method:-(
//   p_->body_P_sensor = body_P_sensor;
//   integrateMeasurement(measuredAcc, measuredOmega, deltaT);
// }
// #endif

//------------------------------------------------------------------------------
// RelativeImuFactor methods
//------------------------------------------------------------------------------
RelativeImuFactor::RelativeImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key extrinsic, Key bias,
                                     const RelativePreintegratedImuMeasurements &pim) : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i,
                                                                                             pose_j, vel_j, extrinsic, bias),
                                                                                        _PIM_(pim)
{
}

//------------------------------------------------------------------------------
NonlinearFactor::shared_ptr RelativeImuFactor::clone() const
{
  return boost::static_pointer_cast<NonlinearFactor>(
      NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
std::ostream &operator<<(std::ostream &os, const RelativeImuFactor &f)
{
  f._PIM_.print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
void RelativeImuFactor::print(const string &s, const KeyFormatter &keyFormatter) const
{
  cout << s << "RelativeImuFactor(" << keyFormatter(this->key1()) << ","
       << keyFormatter(this->key2()) << "," << keyFormatter(this->key3()) << ","
       << keyFormatter(this->key4()) << "," << keyFormatter(this->key5())
       << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
bool RelativeImuFactor::equals(const NonlinearFactor &other, double tol) const
{
  const This *e = dynamic_cast<const This *>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = _PIM_.equals(e->_PIM_, tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
Vector RelativeImuFactor::evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                                        const Pose3 &pose_j, const Vector3 &vel_j,
                                        const Pose3 &extrinsic,
                                        const imuBias::ConstantBias &bias_i,
                                        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3,
                                        boost::optional<Matrix &> H4, boost::optional<Matrix &> H5, boost::optional<Matrix &> H6) const
{
  return _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, extrinsic, bias_i,
                                        H1, H2, H3, H4, H5, H6);
}

//------------------------------------------------------------------------------
// #ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
// RelativeImuFactor::RelativeImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key extrinsic,  Key bias,
//                                      const RelativePreintegratedImuMeasurements &pim,
//                                      const Vector3 &omegaCoriolis, const boost::optional<Pose3> &body_P_sensor,
//                                      const bool use2ndOrderCoriolis) : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i,
//                                                                             pose_j, vel_j, extrinsic, bias),
//                                                                        _PIM_(pim)
// {
//   boost::shared_ptr<RelativePreintegratedImuMeasurements::Params> p = boost::make_shared<
//       RelativePreintegratedImuMeasurements::Params>(pim.p());
//   // p->n_gravity = n_gravity;
//   p->omegaCoriolis = omegaCoriolis;
//   p->body_P_sensor = body_P_sensor;
//   p->use2ndOrderCoriolis = use2ndOrderCoriolis;
//   _PIM_.p_ = p;
// }

// void RelativeImuFactor::Predict(const Pose3 &pose_i, const Vector3 &vel_i,
//                                 Pose3 &pose_j, Vector3 &vel_j, const imuBias::ConstantBias &bias_i,
//                                 const Vector3& n_gravity,
//                                 RelativePreintegratedImuMeasurements &pim,
//                                 const Vector3 &omegaCoriolis, const bool use2ndOrderCoriolis)
// {
//   // use deprecated predict
//   cout << n_gravity << endl;
//   PoseVelocityBias pvb = pim.predict(pose_i, vel_i, bias_i, n_gravity,
//                                      omegaCoriolis, use2ndOrderCoriolis);
//   pose_j = pvb.pose;
//   vel_j = pvb.velocity;
// }
// #endif
//------------------------------------------------------------------------------
// RelativeImuFactor2 methods
//------------------------------------------------------------------------------
RelativeImuFactor2::RelativeImuFactor2(Key state_i, Key state_j, Key bias,
                                       const RelativePreintegratedImuMeasurements &pim) : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), state_i, state_j,
                                                                                               bias),
                                                                                          _PIM_(pim)
{
}

//------------------------------------------------------------------------------
NonlinearFactor::shared_ptr RelativeImuFactor2::clone() const
{
  return boost::static_pointer_cast<NonlinearFactor>(
      NonlinearFactor::shared_ptr(new This(*this)));
}

//------------------------------------------------------------------------------
std::ostream &operator<<(std::ostream &os, const RelativeImuFactor2 &f)
{
  f._PIM_.print("preintegrated measurements:\n");
  os << "  noise model sigmas: " << f.noiseModel_->sigmas().transpose();
  return os;
}

//------------------------------------------------------------------------------
void RelativeImuFactor2::print(const string &s,
                               const KeyFormatter &keyFormatter) const
{
  cout << s << "RelativeImuFactor2(" << keyFormatter(this->key1()) << ","
       << keyFormatter(this->key2()) << "," << keyFormatter(this->key3())
       << ")\n";
  cout << *this << endl;
}

//------------------------------------------------------------------------------
bool RelativeImuFactor2::equals(const NonlinearFactor &other, double tol) const
{
  const This *e = dynamic_cast<const This *>(&other);
  const bool base = Base::equals(*e, tol);
  const bool pim = _PIM_.equals(e->_PIM_, tol);
  return e != nullptr && base && pim;
}

//------------------------------------------------------------------------------
Vector RelativeImuFactor2::evaluateError(const NavState &state_i,
                                         const NavState &state_j,
                                         const imuBias::ConstantBias &bias_i, //
                                         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                         boost::optional<Matrix &> H3) const
{
  return _PIM_.computeError(state_i, state_j, bias_i, H1, H2, H3);
}

//------------------------------------------------------------------------------

} // namespace gtsam
// namespace gtsam
