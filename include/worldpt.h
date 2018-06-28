#ifndef worldpt_H_
#define worldpt_H_

#include <Eigen/Dense>
#include <vector>

class worldpt
{
  public:
    worldpt();
    ~worldpt();
    void setglobal_id(long id);
    long getglobal_id();
    void setvec3_pt(Eigen::Vector3d v);
    void setvec_feature_globalids(std::vector<long> v);
    void addvec_feature_globalid(long i);
    const Eigen::Vector3d& getvec3_pt();
    const std::vector<long>& getvec_feature_globalids();
  private:
    Eigen::Vector3d m_vec3_pt;
    long global_id;
    std::vector<long> m_vector_featurepoint_global_id; // connected feature points
};

#endif