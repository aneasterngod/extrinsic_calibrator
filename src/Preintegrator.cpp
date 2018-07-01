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
    m_mat33_delta_phi_ij.setIdentity();
    m_vec3_prev_omega.setZero();
    m_vec3_curr_omega.setZero();
    m_double_latestTS = 0;
    m_double_elapsedT = 0;
}

Preintegrator::~Preintegrator()
{
}

void Preintegrator::addSignals(double ax, double ay, double az, double rx, double ry, double rz, double ts)
{

    double dt = ts - m_double_latestTS;
    dt = dt / 1000000.0;
    if (m_double_latestTS == 0)
        dt = 0.001;
    double dt2 = dt * dt;

    Sophus::SO3d so3_omega;
    m_vec3_curr_omega << rx, ry, rz;
    Eigen::Vector3d vec3_acc(ax, ay, az);
    so3_omega = Sophus::SO3d::exp(m_vec3_curr_omega * dt);

    Eigen::Vector3d alpha = (m_vec3_curr_omega - m_vec3_prev_omega) / dt;
    Eigen::Matrix3d skew_alpha = Sophus::SO3d::hat(alpha);
    Eigen::Matrix3d skew_omega = Sophus::SO3d::hat(m_vec3_curr_omega);
    Eigen::Matrix3d skew_omega2 = skew_omega * skew_omega;

    // no initial v0 and g0, since later on we will take care of them
    m_vec3_delta_pi_ij = m_vec3_delta_pi_ij + m_vec3_delta_vi_ij * dt + (m_mat33_delta_phi_ij * vec3_acc * dt2) * 0.5;
    m_mat33_delta_pr_ij = m_mat33_delta_pr_ij + m_mat33_delta_vr_ij * dt + (0.5 * m_mat33_delta_phi_ij * (skew_alpha + skew_omega2) * dt2);
    m_vec3_delta_vi_ij = m_vec3_delta_vi_ij + m_mat33_delta_phi_ij * vec3_acc * dt;
    m_mat33_delta_vr_ij = m_mat33_delta_vr_ij + m_mat33_delta_phi_ij * (skew_alpha + skew_omega2) * dt;
    m_mat33_delta_phi_ij = m_mat33_delta_phi_ij * so3_omega.matrix();

    m_double_latestTS = ts;
    m_double_elapsedT = m_double_elapsedT + dt;
}

void Preintegrator::setPrevOmega(Eigen::Vector3d v)
{
    m_vec3_prev_omega = v;
}

const Eigen::Vector3d &Preintegrator::getCurrOmega()
{
    return m_vec3_curr_omega;
}

double Preintegrator::getLatestTS()
{
    return m_double_latestTS;
}

void Preintegrator::setLatestTS(double v)
{
    m_double_latestTS = v;
}

void Preintegrator::printAll()
{
    Sophus::SO3d tmpphi(m_mat33_delta_phi_ij);
    cout << "Elapsed Time: " << m_double_elapsedT << endl;
    cout << "Conventional Delta Phi" << endl;
    cout << tmpphi.angleX() * rad2deg << " " << tmpphi.angleY() * rad2deg << " " << tmpphi.angleZ() * rad2deg << endl;
    cout << "Conventional Delta V" << endl;
    cout << m_vec3_delta_vi_ij.transpose() << endl;
    cout << "Conventional Delta P" << endl;
    cout << m_vec3_delta_pi_ij.transpose() << endl;
    cout << "Relative Delta V" << endl;
    cout << m_mat33_delta_vr_ij << endl;
    cout << "Relative Delta P" << endl;
    cout << m_mat33_delta_pr_ij << endl;
}