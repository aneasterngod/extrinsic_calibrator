#include "FrameData.h"

long __GFID__ = 0;
namespace excalib
{

FrameData::FrameData(std::shared_ptr<GlobalParams> p)
{
    //m_b_FeatureExtracted = false;
    m_shared_ptr_globalparams = p;
    m_shared_ptr_preint_prev2curr = std::shared_ptr<Preintegrator>(new Preintegrator());
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
void FrameData::setImuData(vector<excalib::ImuData> &dat)
{
    m_vector_imudata.assign(dat.begin(), dat.end());
}

void FrameData::clearImuData()
{
    m_vector_imudata.clear();
}

ImageData &FrameData::getImageData()
{
    return m_imagedata;
}

vector<ImuData> &FrameData::getImuData()
{
    return m_vector_imudata;
}

void FrameData::computeFastFeature(bool accum)
{
    if (!accum)
        m_vector_shared_ptr_keypoints.clear();
    bool nonmaxSuppression = true;
    vector<cv::KeyPoint> tmpkeypoints;
    FAST(m_imagedata.getUndistortedImage(), tmpkeypoints, m_shared_ptr_globalparams->getFastThreshold(), nonmaxSuppression);
    for (int i = 0; i < tmpkeypoints.size(); i++)
    {
        tmpkeypoints[i].global_id = __GFID__;
        __GFID__++;
        tmpkeypoints[i].valid = true;

        m_vector_shared_ptr_keypoints.push_back(std::shared_ptr<cv::KeyPoint>(new cv::KeyPoint(tmpkeypoints[i])));
        
        if (accum)
        {
            Eigen::Vector3d ipt(tmpkeypoints[i].pt.x, tmpkeypoints[i].pt.y, 1);
            Eigen::Vector3d wpt = m_shared_ptr_globalparams->getKinv() * ipt;
            std::shared_ptr<Eigen::Vector3d> swpt(new Eigen::Vector3d(wpt));
            m_vector_shared_ptr_vec3_worldpoints.push_back(swpt);
            m_vector_shared_ptr_keypoints.back()->sharedptr_3dpt = swpt;
        }
    }
}

void FrameData::computeGFTFeature(bool accum)
{
    if (!accum)
        m_vector_shared_ptr_keypoints.clear();
    int maxCorners = 1000;
    // qualityLevel – Characterizes the minimal accepted quality of image corners;
    // the value of the parameter is multiplied by the by the best corner quality
    // measure (which is the min eigenvalue, see cornerMinEigenVal() ,
    // or the Harris function response, see cornerHarris() ).
    // The corners, which quality measure is less than the product, will be rejected.
    // For example, if the best corner has the quality measure = 1500,
    // and the qualityLevel=0.01 , then all the corners which quality measure is
    // less than 15 will be rejected.
    double qualityLevel = m_shared_ptr_globalparams->getmineigen();
    // minDistance – The minimum possible Euclidean distance between the returned corners
    double minDistance = m_shared_ptr_globalparams->getMinfeaturedist();
    cv::Mat mask;
    int blockSize = 3;
    bool useHarrisDetector = true;
    double k = 0.04;
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(m_imagedata.getUndistortedImage(), corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k);
    vector<cv::KeyPoint> tmpkeypoints;
    point2keypoint(corners, tmpkeypoints);

    for (int i = 0; i < tmpkeypoints.size(); i++)
    {
        tmpkeypoints[i].global_id = __GFID__;
        __GFID__++;
        tmpkeypoints[i].valid = true;

        m_vector_shared_ptr_keypoints.push_back(std::shared_ptr<cv::KeyPoint>(new cv::KeyPoint(tmpkeypoints[i])));

        if (accum)
        {
            Eigen::Vector3d ipt(tmpkeypoints[i].pt.x, tmpkeypoints[i].pt.y, 1);
            Eigen::Vector3d wpt = m_shared_ptr_globalparams->getKinv() * ipt;
            std::shared_ptr<Eigen::Vector3d> swpt(new Eigen::Vector3d(wpt));
            m_vector_shared_ptr_vec3_worldpoints.push_back(swpt);
            m_vector_shared_ptr_keypoints.back()->sharedptr_3dpt = swpt;
        }
    }
}

void FrameData::keypoint2point(const vector<shared_ptr<cv::KeyPoint>> &src, vector<cv::Point2f> &dst)
{
    dst.clear();
    for (int i = 0; i < (int)src.size(); i++)
    {
        dst.push_back(src[i]->pt);
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
void FrameData::point2keypoint(const vector<cv::Point2f> &src, const vector<shared_ptr<cv::KeyPoint>> &src2, vector<shared_ptr<cv::KeyPoint>> &dst)
{
    dst.clear();
    for (int i = 0; i < (int)src.size(); i++)
    {
        cv::KeyPoint kpt;
        kpt.pt = src[i];
        kpt.global_id = src2[i]->global_id;
        kpt.valid = src2[i]->valid;
        dst.push_back(std::shared_ptr<cv::KeyPoint>(new cv::KeyPoint(kpt)));
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
    double minEigThreshold = m_shared_ptr_globalparams->getmineigen();
    cv::calcOpticalFlowPyrLK(prevframe->getImageData().getUndistortedImage(), getImageData().getUndistortedImage(), pts1, pts2, status, errors, winSize, maxLevel, criteria, flags,
                             minEigThreshold);
    point2keypoint(pts2, prevframe->getPointFeatures(), m_vector_shared_ptr_keypoints);
    int tracked_cnt = 0;
    for (int i = 0; i < m_vector_shared_ptr_keypoints.size(); i++)
    {
        if (status[i] == 1)
        {
            if (prevframe->getPointFeatures()[i]->valid)
            {
                m_vector_shared_ptr_keypoints[i]->valid = true;
                tracked_cnt++;
            }
        }
        else
        {
            m_vector_shared_ptr_keypoints[i]->valid = false;
        }
        m_vector_shared_ptr_keypoints[i]->global_id = prevframe->getPointFeatures()[i]->global_id;
        m_vector_shared_ptr_keypoints[i]->sharedptr_3dpt = prevframe->getPointFeatures()[i]->sharedptr_3dpt;
    }
    if (tracked_cnt < m_shared_ptr_globalparams->getMinimumMaintainedTrackedFeatureNumber())
    {
        //computeFastFeature(true);
        computeGFTFeature(true);
    }

    cout << endl;
}

const vector<shared_ptr<cv::KeyPoint>> &FrameData::getPointFeatures()
{
    return m_vector_shared_ptr_keypoints;
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

const Sophus::SE3d &FrameData::getPose()
{
    return m_se3_pose;
}

void FrameData::setPose(Sophus::SE3d v)
{
    m_se3_pose = v;
}

void FrameData::initWorldpts()
{
    for (int i = 0; i < m_vector_shared_ptr_keypoints.size(); i++)
    {
        Eigen::Vector3d ipt(m_vector_shared_ptr_keypoints[i]->pt.x, m_vector_shared_ptr_keypoints[i]->pt.y, 1);
        Eigen::Vector3d wpt = m_shared_ptr_globalparams->getKinv() * ipt;
        std::shared_ptr<Eigen::Vector3d> swpt(new Eigen::Vector3d(wpt));
        m_vector_shared_ptr_vec3_worldpoints.push_back(swpt);
        m_vector_shared_ptr_keypoints[i]->sharedptr_3dpt = swpt;
    }
}

std::shared_ptr<Preintegrator> FrameData::getPreintegrator(){
    return m_shared_ptr_preint_prev2curr;
}
void FrameData::setPreintegrator(std::shared_ptr<Preintegrator> v){
    m_shared_ptr_preint_prev2curr = v;
}

} // namespace excalib