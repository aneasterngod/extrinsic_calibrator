#include "FrameData.h"
namespace excalib
{

FrameData::FrameData()
{
}

FrameData::~FrameData()
{
}

FrameData &FrameData::operator=(const FrameData &dat)
{
    m_imagedata = dat.m_imagedata;
    m_ar_imudata.assign(dat.m_ar_imudata.begin(), dat.m_ar_imudata.end());
    return *this;
}

FrameData::FrameData(const FrameData &dat)
{
    m_imagedata = dat.m_imagedata;
    m_ar_imudata.assign(dat.m_ar_imudata.begin(), dat.m_ar_imudata.end());
}

void FrameData::setImageData(excalib::ImageData &dat)
{
    m_imagedata = dat;
}
void FrameData::setArImuData(vector<excalib::ImuData> &dat)
{
    m_ar_imudata.assign(dat.begin(), dat.end());
}

ImageData &FrameData::getImageData()
{
    return m_imagedata;
}

vector<ImuData> &FrameData::getArImuData()
{
    return m_ar_imudata;
}

void FrameData::print()
{
    cout << "Image: " << endl;
    m_imagedata.print();
    cout << "Begin: " << endl;
    m_ar_imudata.front().print();
    cout << "End: " << endl;
    m_ar_imudata.back().print();
}

} // namespace excalib