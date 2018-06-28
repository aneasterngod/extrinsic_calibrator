#include "worldpt.h"


worldpt::worldpt(){

}

worldpt::~worldpt(){

}

void worldpt::setglobal_id(long id){
    global_id = id;
}

long worldpt::getglobal_id(){
    return global_id;
}


void worldpt::setvec3_pt(Eigen::Vector3d v){
    m_vec3_pt = v;
}

const Eigen::Vector3d& worldpt::getvec3_pt(){
    return m_vec3_pt;
}

const std::vector<long>& worldpt::getvec_feature_globalids(){
    return m_vector_featurepoint_global_id;
}

void worldpt::setvec_feature_globalids(std::vector<long> v){
    m_vector_featurepoint_global_id = v;
}

void worldpt::addvec_feature_globalid(long i){
    m_vector_featurepoint_global_id.push_back(i);
}
