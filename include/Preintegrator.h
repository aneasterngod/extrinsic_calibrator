#ifndef PREINTEGRATOR_H_
#define PREINTEGRATOR_H_

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <iostream>



class Preintegrator{
public:
    Preintegrator();
    ~Preintegrator();
    void init();    
    void addSignals(float ax, float ay, float az, float rx, float ry, float rz, float dt);
    void printAll();
private:
    Sophus::SO3d m_so3_delta_phi_ij;
    Eigen::Vector3d m_vec3_delta_vi_ij;
    Eigen::Vector3d m_vec3_delta_pi_ij;
    Eigen::Matrix3d m_mat33_delta_vr_ij;
    Eigen::Matrix3d m_mat33_delta_pr_ij;
};
#endif