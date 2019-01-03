#include "gtsam_preintegration.h"

PreintegrationFactor::PreintegrationFactor()
{
}
PreintegrationFactor::~PreintegrationFactor()
{
}

//------------------------------------------------------------------------------
Vector PreintegrationFactor::evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                                           const Pose3 &pose_j, const Vector3 &vel_j,
                                           const Pose3 &extrinsic, const double &scale,
                                           const imuBias::ConstantBias &bias_i, boost::optional<Matrix &> H1,
                                           boost::optional<Matrix &> H2, boost::optional<Matrix &> H3,
                                           boost::optional<Matrix &> H4, boost::optional<Matrix &> H5, boost::optional<Matrix &> H6, boost::optional<Matrix &> H7) const
{
    // return _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, bias_i,
    //                                       H1, H2, H3, H4, H5);
    // extrinsic: R^I_C, t^I_C
    // not bias_g or a, it's delta_bias_g or a, so changes will be applied here instead of bias itself.
    // let's find out how gtsam works


    //꼭 ImuFactor예제를 살펴보라.


    // gtsam::Matrix3 bias_corrected_mat33_delta_phi = m_meas_preint.m_mat33_delta_phi_ij_delta_biasgyro * gtsam::Rot3::Expmap(bias_i.gyroscope()).matrix();
    // gtsam::Matrix3 mat_fphi = bias_corrected_mat33_delta_phi.inverse() * extrinsic.rotation().matrix() * pose_i.rotation().matrix().inverse() * pose_j.rotation().matrix() * extrinsic.inverse().rotation().matrix(); 
    // gtsam::Rot3 gtsamrot_fphi(mat_fphi);
    // gtsam::Vector3 vec3_fphi = gtsam::Rot3::Logmap(gtsamrot_fphi);

    // m_meas_preint.m_vec3_delta_vi_ij + m_meas_preint.m_mat33_delta_vi_ij_delta_biasacc * bias_i.accelerometer()
    //gtsam::Vector3 vec3_DVC_ij = m_meas_preint.m_vec3_delta_vi_ij + m_meas_preint.m_mat33_delta_vr_ij * extrinsic.translation().vector();
    //m_meas_preint.m_mat33_delta_phi_ij.inverse() * extrinsic.rotation().matrix() * pose_i.rotation().inverse() * pose_j.rotation() * extrinsic.rotation().inverse();    

}