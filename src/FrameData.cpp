#include "FrameData.h"

long __GFID__ = 0;
namespace excalib
{

FrameData::FrameData(std::shared_ptr<GlobalParams> &p)
{
    //m_b_FeatureExtracted = false;
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

void FrameData::computeFastFeature(bool accum)
{
    if (!accum)
        m_vector_keypoints.clear();
    bool nonmaxSuppression = true;
    vector<cv::KeyPoint> tmpkeypoints;
    FAST(m_imagedata.getImage(), tmpkeypoints, m_globalparams->getFastThreshold(), nonmaxSuppression);
    for (int i = 0; i < tmpkeypoints.size(); i++)
    {
        tmpkeypoints[i].global_id = __GFID__;
        __GFID__++;
        tmpkeypoints[i].valid = true;
        m_vector_keypoints.push_back(tmpkeypoints[i]);
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
void FrameData::point2keypoint(const vector<cv::Point2f> &src, vector<cv::KeyPoint> &dst)
{
    dst.clear();
    for (int i = 0; i < (int)src.size(); i++)
    {
        cv::KeyPoint kpt;
        kpt.pt = src[i];
        dst.push_back(kpt);
    }
}
void FrameData::point2keypoint(const vector<cv::Point2f> &src, const vector<cv::KeyPoint> &src2, vector<cv::KeyPoint> &dst)
{
    dst.clear();
    for (int i = 0; i < (int)src.size(); i++)
    {
        cv::KeyPoint kpt;
        kpt.pt = src[i];
        kpt.global_id = src2[i].global_id;
        kpt.valid = src2[i].valid;
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
    cv::Size winSize = cv::Size(21, 21);
    int maxLevel = 3;
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.0001);
    int flags = 0;
    double minEigThreshold = 1e-4;
    cv::calcOpticalFlowPyrLK(prevframe->getImageData().getImage(), getImageData().getImage(), pts1, pts2, status, errors, winSize, maxLevel, criteria, flags,
                             minEigThreshold);
    point2keypoint(pts2, prevframe->getPointFeatures(), m_vector_keypoints);
    int tracked_cnt = 0;
    for (int i = 0; i < m_vector_keypoints.size(); i++)
    {
        if (status[i] == 1)
        {
            if (prevframe->getPointFeatures()[i].valid)
            {
                m_vector_keypoints[i].valid = true;
                tracked_cnt++;
            }
        }
        else
        {
            m_vector_keypoints[i].valid = false;
        }
        m_vector_keypoints[i].global_id = prevframe->getPointFeatures()[i].global_id;
        cout << "status[" << i << "]=" << (int)status[i] << endl;
        cout << pts1[i].x << " " << pts1[i].y << "-->" << pts2[i].x << " " << pts2[i].y << endl;
        cout << "prevframe->getPointFeatures()[" << i << "].class_id=" << prevframe->getPointFeatures()[i].class_id << endl;
    }
    if (tracked_cnt < m_globalparams->getMinimumMaintainedTrackedFeatureNumber())
    {
        computeFastFeature(true);
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