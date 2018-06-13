#include "Preintegrator.h"

using namespace std;


#define rad2deg 114.59155902616465
#define deg2rad 0.008726646259971648

Preintegrator::Preintegrator()
{
    m_vec3_delta_vi_ij.setZero();
    m_vec3_delta_pi_ij.setZero();
    m_mat33_delta_vr_ij.setZero();
    m_mat33_delta_pr_ij.setZero();
}

Preintegrator::~Preintegrator()
{
}

void Preintegrator::init()
{
}

void Preintegrator::addSignals(float ax, float ay, float az, float rx, float ry, float rz, float dt)
{
    Sophus::SO3d so3_omega;
    Eigen::Vector3d vec3_omega(rx, ry, rz);
    Eigen::Vector3d vec3_acc(ax, ay, az);
    so3_omega = Sophus::SO3d::exp(vec3_omega);

    m_vec3_delta_pi_ij = m_vec3_delta_pi_ij + m_vec3_delta_vi_ij * dt + (m_so3_delta_phi_ij * vec3_acc * dt * dt) * 0.5;
    m_vec3_delta_vi_ij = m_vec3_delta_vi_ij + m_so3_delta_phi_ij * vec3_acc * dt;
    m_so3_delta_phi_ij = m_so3_delta_phi_ij * so3_omega;
}

void Preintegrator::printAll()
{
    cout << "Conventional Delta Phi" << endl;
    cout << m_so3_delta_phi_ij.angleX() * rad2deg << " " << m_so3_delta_phi_ij.angleY() * rad2deg << " " << m_so3_delta_phi_ij.angleZ() * rad2deg << endl;
    cout << "Conventional Delta V" << endl;
    cout << m_vec3_delta_vi_ij.transpose() << endl;
    cout << "Conventional Delta P" << endl;
    cout << m_vec3_delta_pi_ij.transpose() << endl;
}