#ifndef GlobalParams_H_
#define GlobalParams_H_
#include <iostream>
#include <Eigen/Dense>
 #include <opencv2/core/eigen.hpp>

typedef enum
{
    MATCHING = 1,
    TRACKING = 2
} MatchingType;

class GlobalParams
{
  public:
    GlobalParams()
    {
        m_float_fast_threshold = 10.0f;
    }
    ~GlobalParams()
    {
    }
    float getFastThreshold()
    {
        return m_float_fast_threshold;
    }
    void setFastThreshold(float v){
        m_float_fast_threshold=v;
    }
    MatchingType getMatchingType()
    {
        return m_matchingtype;
    }
    void setMatchingType(MatchingType m)
    {
        m_matchingtype = m;
    }
    int getMinimumMaintainedTrackedFeatureNumber()
    {
        return m_int_minimum_maintained_tracked_feature_numbers;
    }
    void setMinimumMaintainedTrackedFeatureNumber(int v)
    {
        m_int_minimum_maintained_tracked_feature_numbers = v;
    }
    void setmineigen(float v){
        // lower bad quality -> many points
        // higher more good quality
        m_float_mineigen = v;
    }
    float getmineigen(){
        return m_float_mineigen;
    }
    void setMinfeaturedist(int v){
        m_int_mindistfeature = v;
    }
    int getMinfeaturedist(){
        return m_int_mindistfeature;
    }
    void setK(Eigen::Matrix3d K){
        m_mat3_K = K;
        m_mat3_Kinv = m_mat3_K.inverse();
        eigen2cv(m_mat3_K, m_cvmat_K);
        eigen2cv(m_mat3_Kinv, m_cvmat_Kinv);
    }
    void setD(Eigen::Matrix<double, 5, 1>& D){
        m_vec5_D = D;
        eigen2cv(m_vec5_D, m_cvmat_D);
    }
    const Eigen::Matrix3d& getK(){
        return m_mat3_K;        
    }
    const Eigen::Matrix3d& getKinv(){
        return m_mat3_Kinv;        
    }
    
    const Eigen::Matrix<double, 5, 1>& getD(){
        return m_vec5_D;
    }
    const cv::Mat& getKcv(){
        return m_cvmat_K;
    }
    const cv::Mat& getKinvcv(){
        return m_cvmat_Kinv;
    }
    const cv::Mat& getDcv(){
        return m_cvmat_D;
    }
  private:
    float m_float_fast_threshold;
    MatchingType m_matchingtype;
    int m_int_minimum_maintained_tracked_feature_numbers;
    float m_float_mineigen; 
    int m_int_mindistfeature;
    Eigen::Matrix3d m_mat3_K;
    cv::Mat m_cvmat_K;
    Eigen::Matrix3d m_mat3_Kinv;
    cv::Mat m_cvmat_Kinv;
    Eigen::Matrix<double, 5, 1> m_vec5_D;
    cv::Mat m_cvmat_D;
};

#endif