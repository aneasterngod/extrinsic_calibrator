#include "ImuData.h"
namespace excalib
{

ImuData::ImuData()
{
}

ImuData::~ImuData()
{
}

ImuData &ImuData::operator=(const ImuData &dat)
{
    m_s64_timestamp = dat.m_s64_timestamp;
    m_eigenvector3d_acc = dat.m_eigenvector3d_acc;
    m_eigenvector3d_gyro = dat.m_eigenvector3d_gyro;
    return *this;
}

ImuData::ImuData(const ImuData &dat)
{
    m_s64_timestamp = dat.m_s64_timestamp;
    m_eigenvector3d_acc = dat.m_eigenvector3d_acc;
    m_eigenvector3d_gyro = dat.m_eigenvector3d_gyro;
}

void ImuData::setTimestamp(int64_t ts)
{
    m_s64_timestamp = ts;
}

int64_t ImuData::getTimestamp()
{
    return m_s64_timestamp;
}

void ImuData::setAcc(float x, float y, float z)
{
    m_eigenvector3d_acc << x, y, z;
}
Eigen::Vector3d &ImuData::getAcc()
{
    return m_eigenvector3d_acc;
}

void ImuData::setGyro(float x, float y, float z)
{
    m_eigenvector3d_gyro << x, y, z;
}
Eigen::Vector3d &ImuData::getGyro()
{
    return m_eigenvector3d_gyro;
}

void ImuData::print()
{
    cout << "ts: " << m_s64_timestamp << " " << m_eigenvector3d_acc(0) << " " << m_eigenvector3d_acc(1) << " " << m_eigenvector3d_acc(2) << " " << m_eigenvector3d_gyro(0) << " " << m_eigenvector3d_gyro(1) << " " << m_eigenvector3d_gyro(2) << endl;
}
} // namespace excalib