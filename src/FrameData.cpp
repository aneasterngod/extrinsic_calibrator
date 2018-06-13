#include "FrameData.h"
namespace excalib
{

FrameData::FrameData(std::shared_ptr<GlobalParams>& p)
{
    m_b_FeatureExtracted = false;
    m_globalparams = p;
}

FrameData::~FrameData()
{
}

FrameData &FrameData::operator=(const FrameData &dat)
{
    m_imagedata = dat.m_imagedata;
    m_vector_imudata.assign(dat.m_vector_imudata.begin(), dat.m_vector_imudata.end());
    return *this;
}

FrameData::FrameData(const FrameData &dat)
{
    m_imagedata = dat.m_imagedata;
    m_vector_imudata.assign(dat.m_vector_imudata.begin(), dat.m_vector_imudata.end());
}

void FrameData::setImageData(excalib::ImageData &dat)
{
    m_imagedata = dat;
}
void FrameData::setArImuData(vector<excalib::ImuData> &dat)
{
    m_vector_imudata.assign(dat.begin(), dat.end());
}

void FrameData::clearArImuData(){
    m_vector_imudata.clear();
}

ImageData &FrameData::getImageData()
{
    return m_imagedata;
}

vector<ImuData> &FrameData::getArImuData()
{
    return m_vector_imudata;
}

void FrameData::computeFastFeature(){
    if(!m_b_FeatureExtracted){
        bool nonmaxSuppression = true;
	    //FAST(m_imagedata.getImage(), frame->m_framefeature->m_keypoints, fc.fast_threshold, nonmaxSuppression);
    }
}

void FrameData::print()
{
    cout << "Image: " << endl;
    m_imagedata.print();
    cout << "Begin: " << endl;
    m_vector_imudata.front().print();
    cout << "End: " << endl;
    m_vector_imudata.back().print();
}

} // namespace excalib