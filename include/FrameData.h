#ifndef FRAMEDATA_H_
#define FRAMEDATA_H_

#include "excalib_common.h"
#include "ImageData.h"
#include "ImuData.h"
#include "GlobalParams.h"

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
    void print();    
  private:
    excalib::ImageData m_imagedata;
    vector<excalib::ImuData> m_vector_imudata;
    bool m_b_FeatureExtracted;
    std::shared_ptr<GlobalParams> m_globalparams;
};

} // namespace excalib

#endif