#ifndef FRAMEDATA_H_
#define FRAMEDATA_H_

#include "excalib_common.h"
#include "ImageData.h"
#include "ImuData.h"
#include "GlobalParams.h"
#include "Preintegrator.h"
#include <opencv2/opencv.hpp>
//#include <gtsam/geometry/Pose3.h>
#include <sophus/se3.hpp>

class FrameData
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    FrameData(const std::shared_ptr<GlobalParams>& p);
    ~FrameData();
    FrameData & operator=(const FrameData &dat);
    FrameData(const FrameData &dat);
    void setImageData(ImageData& dat);
    void setImuData(vector<ImuData>& dat);
    void clearImuData();
    ImageData& getImageData();
    vector<ImuData>& getImuData();
    void computeFastFeature(bool accum = false, bool worldptgen = false);
    void computeGFTFeature(bool accum = false, bool worldptgen = false);
    static void keypoint2point(const vector<shared_ptr<cv::KeyPoint>>& src, vector<cv::Point2f>& dst) ;
    static void point2keypoint(const vector<cv::Point2f>& src, vector<cv::KeyPoint>& dst) ;
    static void point2keypoint(const vector<cv::Point2f> &src, vector<shared_ptr<cv::KeyPoint> > &dst);
    static void point2keypoint(const vector<cv::Point2f>& src, const vector<shared_ptr<cv::KeyPoint>>& src2, vector<shared_ptr<cv::KeyPoint>>& dst);
    void convertRawpose2SE3();
    bool doTracking(std::shared_ptr<FrameData> prevframe);
    const vector<shared_ptr<cv::KeyPoint>>& getPointFeatures();        
    void setPointFeatures(vector<shared_ptr<cv::KeyPoint>>& dst);
    void print();    
    const Sophus::SE3d& getPose();
    double* getRawPose();
    void clonePose(std::shared_ptr<FrameData> fp);
    void setPose(Sophus::SE3d v);
    // void setGtsampose(const gtsam::Vector6& pose);
    // const gtsam::Pose3& getGtsampose();
    // void initWorldpts();
    std::shared_ptr<Preintegrator> getPreintegrator();
    void setPreintegrator(std::shared_ptr<Preintegrator> v);
    long getID(){
      return m_id;      
    }
    void setID(long id){
      m_id = id;
    }

  private:
    long m_id;
    ImageData m_imagedata;
    vector<ImuData> m_vector_imudata;
    std::shared_ptr<GlobalParams> m_shared_ptr_globalparams;
    vector<shared_ptr<cv::KeyPoint>> m_vector_shared_ptr_keypoints;
    vector<shared_ptr<double>> m_vector_shared_ptr_worldpoints;
    std::shared_ptr<FrameData> m_shared_ptr_keyframe;
    Sophus::SE3d m_se3_pose;
    double m_doublearray_pose[6];
    std::shared_ptr<Preintegrator> m_shared_ptr_preint_prev2curr;


  //  gtsam::Pose3 m_gtsam_pose;
};


#endif