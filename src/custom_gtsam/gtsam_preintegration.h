/* GTSAM includes */
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/base/debug.h>
#include <gtsam/geometry/Pose3.h>

#include "Preintegrator.h"
// maximum 6, so
// 1 pose i (p,r)
// 2 vel j (v)
// 3 pose j
// 4 vel j
// 5 pose extrinsic calibration
// 6 scale
// 7 bias
// scale? no it's implied in the pose
using namespace gtsam;

class PreintegrationFactor : public gtsam::NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, double, gtsam::imuBias::ConstantBias>
{
  public:
    PreintegrationFactor();
    ~PreintegrationFactor();
    Vector evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                         const Pose3 &pose_j, const Vector3 &vel_j,
                         const Pose3 &extrinsic, const double &scale,
                         const imuBias::ConstantBias &bias_i, boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                         boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none, boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none, boost::optional<Matrix &> H7 = boost::none) const;

  private:
    Preintegrator m_meas_preint;
};