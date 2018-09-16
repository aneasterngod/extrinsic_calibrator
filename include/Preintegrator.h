#ifndef PREINTEGRATOR_H_
#define PREINTEGRATOR_H_

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <iostream>
#include "excalib_common.h"
class Preintegrator
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    Preintegrator();
    ~Preintegrator();
    void addSignals(double ax, double ay, double az, double rx, double ry, double rz, double ts);
    void setPrevOmega(const Eigen::Vector3d& v);
    const Eigen::Vector3d &getCurrOmega();
    double getLatestTS();
    void setLatestTS(double v);
    void printAll();
    int getNumSignals();
    void clear();
  private:
    Eigen::Vector3d m_vec3_delta_vi_ij;
    Eigen::Vector3d m_vec3_delta_pi_ij;
    Eigen::Matrix3d m_mat33_delta_phi_ij;
    Eigen::Matrix3d m_mat33_delta_vr_ij;
    Eigen::Matrix3d m_mat33_delta_pr_ij;

    Eigen::Vector3d m_vec3_prev_omega;
    Eigen::Vector3d m_vec3_curr_omega;
    double m_double_elapsedT;
    double m_double_latestTS;
    int m_int_numberofsignals;
};
#endif