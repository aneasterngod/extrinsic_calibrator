#ifndef FRAMEDATA_H_
#define FRAMEDATA_H_

#include "excalib_common.h"
#include "ImageData.h"
#include "ImuData.h"
#include "GlobalParams.h"
#include "worldpt.h"
#include "Preintegrator.h"
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace excalib
{

class FrameData
{
  public:
    FrameData(std::shared_ptr<GlobalParams> p);
    ~FrameData();
    FrameData & operator=(const FrameData &dat);
    FrameData(const FrameData &dat);
    void setImageData(excalib::ImageData& dat);
    void setImuData(vector<excalib::ImuData>& dat);
    void clearImuData();
    excalib::ImageData& getImageData();
    vector<excalib::ImuData>& getImuData();
    void computeFastFeature(bool accum = false);
    void computeGFTFeature(bool accum = false);
    void keypoint2point(const vector<shared_ptr<cv::KeyPoint>>& src, vector<cv::Point2f>& dst) ;
    void point2keypoint(const vector<cv::Point2f>& src, vector<cv::KeyPoint>& dst) ;
    void point2keypoint(const vector<cv::Point2f>& src, const vector<shared_ptr<cv::KeyPoint>>& src2, vector<shared_ptr<cv::KeyPoint>>& dst);
    void doTracking(std::shared_ptr<excalib::FrameData> prevframe);
    const vector<shared_ptr<cv::KeyPoint>>& getPointFeatures();    
    void print();    
    const Sophus::SE3d& getPose();
    void setPose(Sophus::SE3d v);
    void initWorldpts();
    std::shared_ptr<Preintegrator> getPreintegrator();
    void setPreintegrator(std::shared_ptr<Preintegrator> v);
  private:
    excalib::ImageData m_imagedata;
    vector<excalib::ImuData> m_vector_imudata;
    std::shared_ptr<GlobalParams> m_shared_ptr_globalparams;
    vector<shared_ptr<cv::KeyPoint>> m_vector_shared_ptr_keypoints;
    vector<shared_ptr<Eigen::Vector3d>> m_vector_shared_ptr_vec3_worldpoints;
    std::shared_ptr<FrameData> m_shared_ptr_keyframe;
    Sophus::SE3d m_se3_pose;
    std::shared_ptr<Preintegrator> m_shared_ptr_preint_prev2curr;
};

} // namespace excalib

#endif