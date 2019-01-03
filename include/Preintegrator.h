#ifndef PREINTEGRATOR_H_
#define PREINTEGRATOR_H_

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/NoiseModel.h>


using namespace gtsam;
class Preintegrator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  Preintegrator();
  ~Preintegrator();

  typedef imuBias::ConstantBias Bias;


  NavState deltaXij() const  { return deltaXij_; }
  Rot3 deltaRij() const  { return deltaXij_.attitude(); }
  Vector3 deltaPij() const  { return deltaXij_.position(); }
  Vector3 deltaVij() const  { return deltaXij_.velocity(); }
  Matrix3 deltaRPij() const  { return deltaRPij_; }
  Matrix3 deltaRVij() const  { return deltaRVij_; }

  Matrix3 delRdelBiasOmega() const { return delRdelBiasOmega_; }
  Matrix3 delPdelBiasAcc() const { return delPdelBiasAcc_; }
  Matrix3 delPdelBiasOmega() const { return delPdelBiasOmega_; }
  Matrix3 delVdelBiasAcc() const { return delVdelBiasAcc_; }
  Matrix3 delVdelBiasOmega() const { return delVdelBiasOmega_; }
  Matrix3 delRPdelBiasOmega() const { return delRPdelBiasOmega_; }
  Matrix3 delRVdelBiasOmega() const { return delRVdelBiasOmega_; }
  Matrix9 Sigma_ij() const { return Sigma_ij_; }
  Matrix6 Sigma_noise() const { return Sigma_noise_; }
  Bias biasHat() const {return biasHat_;}
  double deltaTij() const {return deltaTij_;}

protected:
  NavState deltaXij_;
  Matrix3 deltaRPij_;
  Matrix3 deltaRVij_;

  Matrix3 delRdelBiasOmega_;  ///< Jacobian of preintegrated rotation w.r.t. angular rate bias
  Matrix3 delPdelBiasAcc_;    ///< Jacobian of preintegrated position w.r.t. acceleration bias
  Matrix3 delPdelBiasOmega_;  ///< Jacobian of preintegrated position w.r.t. angular rate bias
  Matrix3 delVdelBiasAcc_;    ///< Jacobian of preintegrated velocity w.r.t. acceleration bias
  Matrix3 delVdelBiasOmega_;  ///< Jacobian of preintegrated velocity w.r.t. angular rate bias
  Matrix3 delRPdelBiasOmega_; ///< Jacobian of relative preintegrated position w.r.t. angular rate bias
  Matrix3 delRVdelBiasOmega_; ///< Jacobian of relative preintegrated velocity w.r.t. angular rate bias
  Matrix9 Sigma_ij_;          ///< Noise of the relative preintegration
  Matrix6 Sigma_noise_;       ///< Given noise
  Vector3 prevOmega_;
  Bias biasHat_;
  double deltaTij_;
};
#endif