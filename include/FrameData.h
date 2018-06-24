#ifndef FRAMEDATA_H_
#define FRAMEDATA_H_

#include "excalib_common.h"
#include "ImageData.h"
#include "ImuData.h"
#include "GlobalParams.h"
#include <opencv2/opencv.hpp>

namespace excalib
{

class FrameData
{
  public:
    FrameData(std::shared_ptr<GlobalParams>& p);
    ~FrameData();
    FrameData & operator=(const FrameData &dat);
    FrameData(const FrameData &dat);
    void setImageData(excalib::ImageData& dat);
    void setArImuData(vector<excalib::ImuData>& dat);
    void clearArImuData();
    excalib::ImageData& getImageData();
    vector<excalib::ImuData>& getArImuData();
    void computeFastFeature();
    void keypoint2point(const vector<cv::KeyPoint>& src, vector<cv::Point2f>& dst) ;
    void point2keypoint(const vector<cv::Point2f>& src, vector<cv::KeyPoint>& dst) ;
    void doTracking(std::shared_ptr<excalib::FrameData> prevframe);
    const vector<cv::KeyPoint>& getPointFeatures();    
    void print();    
  private:
    excalib::ImageData m_imagedata;
    vector<excalib::ImuData> m_vector_imudata;
    vector<cv::KeyPoint> m_vector_keypoints;    
    bool m_b_FeatureExtracted;
    std::shared_ptr<GlobalParams> m_globalparams;
};

} // namespace excalib

#endif