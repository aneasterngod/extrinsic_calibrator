/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RelativeImuFactor.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/base/debug.h>

namespace gtsam
{

typedef ManifoldPreintegration PreintegrationType;

/*
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, "Eliminating
 * conditionally independent sets in factor graphs: a unifying perspective based
 * on smart factors", Int. Conf. on Robotics and Automation (ICRA), 2014.
 *
 * C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
 * Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
 * Robotics: Science and Systems (RSS), 2015.
 *
 * REFERENCES:
 * [1] G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups",
 *     Volume 2, 2008.
 * [2] T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for
 *     High-Dynamic Motion in Built Environments Without Initial Conditions",
 *     TRO, 28(1):61-76, 2012.
 * [3] L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor:
 *     Computation of the Jacobian Matrices", Tech. Report, 2013.
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
 *     Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
 *     Robotics: Science and Systems (RSS), 2015.
 */

/**
 * RelativePreintegratedImuMeasurements accumulates (integrates) the IMU measurements
 * (rotation rates and accelerations) and the corresponding covariance matrix.
 * The measurements are then used to build the Preintegrated IMU factor.
 * Integration is done incrementally (ideally, one integrates the measurement
 * as soon as it is received from the IMU) so as to avoid costly integration
 * at time of factor construction.
 *
 * @addtogroup SLAM
 */
class RelativePreintegratedImuMeasurements : public PreintegrationType
{

  friend class RelativeImuFactor;
  friend class RelativeImuFactor2;

public:
  void setLeverarm(const Vector3 &tic);
  Matrix3 deltaRPij() const { return deltaRPij_; }
  Matrix3 deltaRVij() const { return deltaRVij_; }
  Matrix3 delRPdelBiasOmega() const { return delRPdelBiasOmega_; }
  Matrix3 delRVdelBiasOmega() const { return delRVdelBiasOmega_; }
  void setPrevOmega(const Vector3 &w);
  Matrix3 Jr(const Vector3 &x);
  Matrix3 Jrinv(const Vector3 &x);
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i, OptionalJacobian<9, 6> H) const override;
  Vector9 computeErrorAndJacobians(const Pose3& pose_i,
    const Vector3& vel_i, const Pose3& pose_j, const Vector3& vel_j,
    const Pose3& extrinsic,
    const imuBias::ConstantBias& bias_i, 
    OptionalJacobian<9, 6> H1, OptionalJacobian<9, 3> H2, OptionalJacobian<9, 6> H3,
    OptionalJacobian<9, 3> H4, OptionalJacobian<9, 6> H5, OptionalJacobian<9, 6> H6) const;

protected:
  Vector3 TIC_;
  Matrix3 deltaRPij_;
  Matrix3 deltaRVij_;
  Matrix3 delRPdelBiasOmega_; ///< Jacobian of relative preintegrated position w.r.t. angular rate bias
  Matrix3 delRVdelBiasOmega_; ///< Jacobian of relative preintegrated velocity w.r.t. angular rate bias
private:
  Vector3 prevOmega_;



protected:
  Matrix9 preintMeasCov_; ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
  ///< (first-order propagation from *measurementCovariance*).

public:
  /// Default constructor for serialization and Cython wrapper
  RelativePreintegratedImuMeasurements()
  {
    preintMeasCov_.setZero();
  }

  /**
   *  Constructor, initializes the class with no measurements
   *  @param bias Current estimate of acceleration and rotation rate biases
   *  @param p    Parameters, typically fixed in a single application
   */
  RelativePreintegratedImuMeasurements(const boost::shared_ptr<PreintegrationParams> &p,
                                       const imuBias::ConstantBias &biasHat = imuBias::ConstantBias()) : PreintegrationType(p, biasHat)
  {
    preintMeasCov_.setZero();
  }

  /**
  *  Construct preintegrated directly from members: base class and preintMeasCov
  *  @param base               PreintegrationType instance
  *  @param preintMeasCov      Covariance matrix used in noise model.
  */
  RelativePreintegratedImuMeasurements(const PreintegrationType &base, const Matrix9 &preintMeasCov)
      : PreintegrationType(base),
        preintMeasCov_(preintMeasCov)
  {
  }

  /// print
  void print(const std::string &s = "Preintegrated Measurements:") const override;

  /// equals
  bool equals(const RelativePreintegratedImuMeasurements &expected, double tol = 1e-9) const;

  /// Re-initialize RelativePreintegratedImuMeasurements
  void resetIntegration() override;

  /**
   * Add a single IMU measurement to the preintegration.
   * @param measuredAcc Measured acceleration (in body frame, as given by the sensor)
   * @param measuredOmega Measured angular velocity (as given by the sensor)
   * @param dt Time interval between this and the last IMU measurement
   */
  void integrateMeasurement(const Vector3 &measuredAcc,
                            const Vector3 &measuredOmega, const double dt) override;

  /// Add multiple measurements, in matrix columns
  void integrateMeasurements(const Matrix &measuredAccs, const Matrix &measuredOmegas,
                             const Matrix &dts);

  /// Return pre-integrated measurement covariance
  Matrix preintMeasCov() const { return preintMeasCov_; }

// #ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
//   /// @deprecated constructor
//   /// NOTE(frank): assumes Z-Down convention, only second order integration supported
//   RelativePreintegratedImuMeasurements(const imuBias::ConstantBias &biasHat,
//                                        const Matrix3 &measuredAccCovariance,
//                                        const Matrix3 &measuredOmegaCovariance,
//                                        const Matrix3 &integrationErrorCovariance,
//                                        bool use2ndOrderIntegration = true);

//   /// @deprecated version of integrateMeasurement with body_P_sensor
//   /// Use parameters instead
//   void integrateMeasurement(const Vector3 &measuredAcc,
//                             const Vector3 &measuredOmega, double dt,
//                             boost::optional<Pose3> body_P_sensor);
// #endif

private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/)
  {
    namespace bs = ::boost::serialization;
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationType);
    ar &bs::make_nvp("preintMeasCov_", bs::make_array(preintMeasCov_.data(), preintMeasCov_.size()));
  }
};

/**
 * RelativeImuFactor is a 5-ways factor involving previous state (pose and velocity of
 * the vehicle at previous time step), current state (pose and velocity at
 * current time step), and the bias estimate. Following the preintegration
 * scheme proposed in [2], the RelativeImuFactor includes many IMU measurements, which
 * are "summarized" using the RelativePreintegratedImuMeasurements class.
 * Note that this factor does not model "temporal consistency" of the biases
 * (which are usually slowly varying quantities), which is up to the caller.
 * See also CombinedRelativeImuFactor for a class that does this for you.
 *
 * @addtogroup SLAM
 */
class RelativeImuFactor : public NoiseModelFactor6<Pose3, Vector3, Pose3, Vector3, Pose3, 
                                                   imuBias::ConstantBias>
{

private:
  typedef RelativeImuFactor This;
  typedef NoiseModelFactor6<Pose3, Vector3, Pose3, Vector3, Pose3, 
                            imuBias::ConstantBias> Base;

  RelativePreintegratedImuMeasurements _PIM_;

public:
  /** Shorthand for a smart pointer to a factor */
#if !defined(_MSC_VER) && __GNUC__ == 4 && __GNUC_MINOR__ > 5
  typedef typename boost::shared_ptr<RelativeImuFactor> shared_ptr;
#else
  typedef boost::shared_ptr<RelativeImuFactor> shared_ptr;
#endif

  /** Default constructor - only use for serialization */
  RelativeImuFactor() {}

  /**
   * Constructor
   * @param pose_i Previous pose key
   * @param vel_i  Previous velocity key
   * @param pose_j Current pose key
   * @param vel_j  Current velocity key
   * @param bias   Previous bias key
   */
  RelativeImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key extrinsic, Key bias,
                    const RelativePreintegratedImuMeasurements &preintegratedMeasurements);

  virtual ~RelativeImuFactor()
  {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const;

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const RelativeImuFactor &);
  virtual void print(const std::string &s, const KeyFormatter &keyFormatter =
                                               DefaultKeyFormatter) const;
  virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const;
  /// @}

  /** Access the preintegrated measurements. */

  const RelativePreintegratedImuMeasurements &preintegratedMeasurements() const
  {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                       const Pose3 &pose_j, const Vector3 &vel_j,
                       const Pose3 &extrinsic, 
                       const imuBias::ConstantBias &bias_i, 
                       boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                       boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none, 
                       boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none) const;

// #ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
//   /// @deprecated typename
//   typedef RelativePreintegratedImuMeasurements PreintegratedMeasurements;

//   /// @deprecated constructor, in the new one gravity, coriolis settings are in PreintegrationParams
//   RelativeImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key extrinsic, Key bias,
//                     const PreintegratedMeasurements &preintegratedMeasurements,
//                     const Vector3 &omegaCoriolis,
//                     const boost::optional<Pose3> &body_P_sensor = boost::none,
//                     const bool use2ndOrderCoriolis = false);

//   /// @deprecated use PreintegrationBase::predict,
//   /// in the new one gravity, coriolis settings are in PreintegrationParams
//   static void Predict(const Pose3 &pose_i, const Vector3 &vel_i, Pose3 &pose_j,
//                       Vector3 &vel_j, const imuBias::ConstantBias &bias_i,
//                       const Vector3& n_gravity,
//                       PreintegratedMeasurements &pim, 
//                       const Vector3 &omegaCoriolis, const bool use2ndOrderCoriolis = false);
// #endif

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/)
  {
    ar &boost::serialization::make_nvp("NoiseModelFactor5",
                                       boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(_PIM_);
  }
};
// class RelativeImuFactor

/**
 * RelativeImuFactor2 is a ternary factor that uses NavStates rather than Pose/Velocity.
 * @addtogroup SLAM
 */
class RelativeImuFactor2 : public NoiseModelFactor3<NavState, NavState, imuBias::ConstantBias>
{
private:
  typedef RelativeImuFactor2 This;
  typedef NoiseModelFactor3<NavState, NavState, imuBias::ConstantBias> Base;

  RelativePreintegratedImuMeasurements _PIM_;

public:
  /** Default constructor - only use for serialization */
  RelativeImuFactor2() {}

  /**
   * Constructor
   * @param state_i Previous state key
   * @param state_j Current state key
   * @param bias    Previous bias key
   */
  RelativeImuFactor2(Key state_i, Key state_j, Key bias,
                     const RelativePreintegratedImuMeasurements &preintegratedMeasurements);

  virtual ~RelativeImuFactor2()
  {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const;

  /// @name Testable
  /// @{
  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const RelativeImuFactor2 &);
  virtual void print(const std::string &s, const KeyFormatter &keyFormatter =
                                               DefaultKeyFormatter) const;
  virtual bool equals(const NonlinearFactor &expected, double tol = 1e-9) const;
  /// @}

  /** Access the preintegrated measurements. */

  const RelativePreintegratedImuMeasurements &preintegratedMeasurements() const
  {
    return _PIM_;
  }

  /** implement functions needed to derive from Factor */

  /// vector of errors
  Vector evaluateError(const NavState &state_i, const NavState &state_j,
                       const imuBias::ConstantBias &bias_i, //
                       boost::optional<Matrix &> H1 = boost::none,
                       boost::optional<Matrix &> H2 = boost::none,
                       boost::optional<Matrix &> H3 = boost::none) const;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/)
  {
    ar &boost::serialization::make_nvp("NoiseModelFactor3",
                                       boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(_PIM_);
  }
};
// class RelativeImuFactor2

template <>
struct traits<RelativePreintegratedImuMeasurements> : public Testable<RelativePreintegratedImuMeasurements>
{
};

template <>
struct traits<RelativeImuFactor> : public Testable<RelativeImuFactor>
{
};

template <>
struct traits<RelativeImuFactor2> : public Testable<RelativeImuFactor2>
{
};

} // namespace gtsam
