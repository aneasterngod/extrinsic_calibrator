#ifndef EXCALIBIMUDATA_H_
#define EXCALIBIMUDATA_H_

#include "excalib_common.h"

class ImuData
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    ImuData();
    ~ImuData();
    ImuData& operator=(const ImuData& dat);
    ImuData(const ImuData &dat);
    void setTimestamp(int64_t ts);
    int64_t getTimestamp();
    void setAcc(float x, float y, float z);
    Eigen::Vector3d &getAcc();
    void setGyro(float x, float y, float z);
    Eigen::Vector3d &getGyro();

    void print();

  private:
    int64_t m_s64_timestamp;
    Eigen::Vector3d m_vec3d_acc;
    Eigen::Vector3d m_vec3d_gyro;
};


#endif