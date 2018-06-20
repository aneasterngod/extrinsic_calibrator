#include "FrameData.h"

long __GFID__ = 0;
namespace excalib
{

FrameData::FrameData(std::shared_ptr<GlobalParams> &p)
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

void FrameData::clearArImuData()
{
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

void FrameData::computeFastFeature()
{
    if (!m_b_FeatureExtracted)
    {
        bool nonmaxSuppression = true;
        FAST(m_imagedata.getImage(), m_vector_keypoints, m_globalparams->getFastThreshold(), nonmaxSuppression);
        for (int i = 0; i < m_vector_keypoints.size(); i++)
        {
            m_vector_keypoints[i].global_id = __GFID__;
            __GFID__++;
        }
    }
}
void FrameData::keypoint2point(const vector<cv::KeyPoint> &src, vector<cv::Point2f> &dst)
{
    dst.clear();
    for (int i = 0; i < (int)src.size(); i++)
    {
        dst.push_back(src[i].pt);
    }
}
void FrameData::point2keypoint(const vector<cv::Point2f>& src, vector<cv::KeyPoint>& dst) {
	dst.clear();
	for (int i = 0; i < (int) src.size(); i++) {
		cv::KeyPoint kpt;
		kpt.pt = src[i];
		dst.push_back(kpt);
	}
}

void FrameData::doTracking(std::shared_ptr<excalib::FrameData> prevframe)
{
    vector<uchar> status;
    vector<float> errors;
    vector<cv::Point2f> pts1;
    vector<cv::Point2f> pts2;
    keypoint2point(prevframe->getPointFeatures(), pts1);
    pts2.resize(pts1.size());
    cv::Size winSize = cv::Size(15, 15);
    int maxLevel = 2;
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.03);
    int flags = 0;
    double minEigThreshold = 1e-4;
    cv::calcOpticalFlowPyrLK(prevframe->getImageData().getImage(), getImageData().getImage(), pts1, pts2, status, errors, winSize, maxLevel, criteria, cv::OPTFLOW_USE_INITIAL_FLOW,
                             minEigThreshold);
	point2keypoint(pts2, m_vector_keypoints);
    for(int i=0;i<m_vector_keypoints.size();i++){                
        m_vector_keypoints[i].global_id = prevframe->getPointFeatures()[i].global_id;
        cout << "status[" << i << "]=" << (int)status[i] << endl;
        cout << "prevframe->getPointFeatures()[" << i << "].class_id=" << prevframe->getPointFeatures()[i].class_id << endl;
    }
    cout << endl;
}

const vector<cv::KeyPoint> &FrameData::getPointFeatures()
{
    return m_vector_keypoints;
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