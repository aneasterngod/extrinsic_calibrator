#ifndef Frame_H_
#define Frame_H_

#include <vector>
#include <gtsam/geometry/Pose3.h>

#include "Preintegrator.h"
#include "ImageData.h"
#include "ImuData.h"
#include "Parameters.h"

extern long GLOBALFEATUREID;

class Frame
{
public:
  Frame();
  ~Frame();
  void setImageData(const ImageData &dat);
  ImageData &getImageData();
  std::vector<ImuData>& getImuData();
  void setImuData(const vector<ImuData> &dat);
  void setID(long id)
  {
    m_s64Id = id;
  }
  long getID()
  {
    return m_s64Id;
  }
  void clearImuData();
  const gtsam::Pose3 &getGtsamPose();
  void setGtsamPose(const gtsam::Pose3 &pose);
  void setKeyframe(bool s)
  {
    m_bKeyframe = s;
  }
  //////////////////////////////////////////////////////////////////
  // Feature
  void extractFASTFeaturePoints(const Parameters &params);
  void extractGFTTFeaturePoints(const Parameters &params);
  bool doTrack(const Parameters &params, std::shared_ptr<Frame> prev);
  bool doMatch(const Parameters &params, std::shared_ptr<Frame> prev);

  static void keypoint2point(const std::list<std::shared_ptr<cv::KeyPoint>> &src, vector<cv::Point2f> &dst);

  std::list<std::shared_ptr<cv::KeyPoint>> &getPointfeatures();

  Preintegrator& getPreintegrator();

  std::shared_ptr<Frame> m_ptrNext;
  std::shared_ptr<Frame> m_ptrPrev;

private:
  // frame
  long m_s64Id;
  bool m_bKeyframe;

  ImageData m_imagedata;
  std::vector<ImuData> m_vector_imudata_prev2curr;

  // features
  std::list<std::shared_ptr<cv::KeyPoint>> m_list_keypoints;
  cv::Mat m_descriptors;

  // poses
  gtsam::Pose3 m_gtsampose_currpose;

  // preintegration
  Preintegrator m_preintegrator_prev2curr;
};

#endif