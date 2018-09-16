#include "Frame.h"

long GLOBALFEATUREID = 0;

Frame::Frame()
{
}

Frame::~Frame()
{
}

void Frame::setImageData(const ImageData &dat)
{
    m_imagedata = dat;
}
ImageData &Frame::getImageData()
{
    return m_imagedata;
}
std::vector<ImuData>& Frame::getImuData(){
    return m_vector_imudata_prev2curr;
}
void Frame::setImuData(const vector<ImuData> &dat)
{
    m_vector_imudata_prev2curr.assign(dat.begin(), dat.end());
}

void Frame::clearImuData()
{
    m_vector_imudata_prev2curr.clear();
}

const gtsam::Pose3 &Frame::getGtsamPose()
{
    return m_gtsampose_currpose;
}

void Frame::setGtsamPose(const gtsam::Pose3 &pose)
{
    m_gtsampose_currpose = pose;
}

//////////////////////////////////////////////////////////////////
// Feature
void Frame::extractFASTFeaturePoints(const Parameters &params)
{
    vector<cv::KeyPoint> tmpkeypoints;
    FAST(m_imagedata.getUndistortedImage(), tmpkeypoints, params.m_stParam.pF.pFast.m_f32Threshold, params.m_stParam.pF.pFast.m_bNMS);

    for (int i = 0; i < tmpkeypoints.size(); i++)
    {
        tmpkeypoints[i].global_id = GLOBALFEATUREID;
        GLOBALFEATUREID++;

        uchar b = m_imagedata.getUndistortedImage().at<uchar>(tmpkeypoints[i].pt.y, tmpkeypoints[i].pt.x);
        uchar g = m_imagedata.getUndistortedImage().at<uchar>(tmpkeypoints[i].pt.y, tmpkeypoints[i].pt.x);
        uchar r = m_imagedata.getUndistortedImage().at<uchar>(tmpkeypoints[i].pt.y, tmpkeypoints[i].pt.x);
        tmpkeypoints[i].color = cv::Scalar(b, g, r);

        Eigen::Vector3d ipt(tmpkeypoints[i].pt.x, tmpkeypoints[i].pt.y, 1);
        Eigen::Vector3d wpt = params.m_stParam.pC.m_mat3_Kinv * ipt;
        gtsam::Point3 gtsamwpt = m_gtsampose_currpose * wpt;
        tmpkeypoints[i].landmark = std::shared_ptr<Eigen::Vector3d>(new Eigen::Vector3d());
        (*tmpkeypoints[i].landmark.get()) << gtsamwpt.x(), gtsamwpt.y(), gtsamwpt.z();
        tmpkeypoints[i].history = 1;

        std::shared_ptr<cv::KeyPoint> ptrkpt(new cv::KeyPoint(tmpkeypoints[i]));
        // accumulate m_vector_featurepoints
        m_list_keypoints.push_back(ptrkpt);
    }
}

void Frame::extractGFTTFeaturePoints(const Parameters &params)
{
    int16_t s16MaxCorners = params.m_stParam.pF.pGFTT.m_s16Maxfeaturepoints;
    float f32QualityLevel = params.m_stParam.pF.pGFTT.m_f32QualityLevel;
    float f32MinDistance = params.m_stParam.pF.pGFTT.m_s32Mindistfeature;
    cv::Mat mask;
    int8_t s8BlockSize = params.m_stParam.pF.pGFTT.m_s8BlockSize;
    bool bUseHarrisDetector = true;
    float f32K = 0.04;
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(m_imagedata.getUndistortedImage(), corners, s16MaxCorners, f32QualityLevel, f32MinDistance, mask, s8BlockSize, bUseHarrisDetector, f32K);

    for (int i = 0; i < corners.size(); i++)
    {
        std::shared_ptr<cv::KeyPoint> ptrkpt(new cv::KeyPoint());

        ptrkpt->global_id = GLOBALFEATUREID;
        GLOBALFEATUREID++;

        uchar b = m_imagedata.getUndistortedImage().at<uchar>(corners[i].y, corners[i].x);
        uchar g = m_imagedata.getUndistortedImage().at<uchar>(corners[i].y, corners[i].x);
        uchar r = m_imagedata.getUndistortedImage().at<uchar>(corners[i].y, corners[i].x);
        ptrkpt->color = cv::Scalar(b, g, r);
        ptrkpt->frameid = getID();
//        Eigen::Vector3d ipt(corners[i].x, corners[i].y, 1);
//        Eigen::Vector3d wpt = params.m_stParam.pC.m_mat3_Kinv * ipt;
//        gtsam::Point3 gtsamwpt = m_gtsampose_currpose * wpt;
//        ptrkpt->landmark = std::shared_ptr<Eigen::Vector3d>(new Eigen::Vector3d());
//        (*ptrkpt->landmark.get()) << gtsamwpt.x(), gtsamwpt.y(), gtsamwpt.z();
        ptrkpt->pt = corners[i];
        ptrkpt->history = 1;
        // accumulate m_vector_featurepoints
        m_list_keypoints.push_back(ptrkpt);
    }
}

void Frame::keypoint2point(const std::list<std::shared_ptr<cv::KeyPoint>> &src, vector<cv::Point2f> &dst)
{
    dst.clear();
    std::list<std::shared_ptr<cv::KeyPoint>>::const_iterator itr = src.begin();
    for (itr = src.begin(); itr != src.end(); itr++)
    {
        cv::Point2f pt;
        pt = (*itr)->pt;
        dst.push_back(pt);
    }
}

bool Frame::doTrack(const Parameters &params, std::shared_ptr<Frame> prev)
{
    vector<uchar> status;
    vector<float> errors;
    vector<cv::Point2f> pts1;
    vector<cv::Point2f> pts2;
    Frame::keypoint2point(prev->getPointfeatures(), pts1);
    pts2.resize(pts1.size());
    cv::Size winSize = cv::Size(21, 21);
    int maxLevel = 3;
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.0001);
    int flags = 0;
    double minEigThreshold = params.m_stParam.pF.pKLT.m_f32MinEigen;
    cv::calcOpticalFlowPyrLK(prev->getImageData().getUndistortedImage(), getImageData().getUndistortedImage(), pts1, pts2, status, errors, winSize, maxLevel, criteria, flags,
                             minEigThreshold);
    
    // // fundamental filter
    vector<uchar> inliers(pts1.size(), 0);
    cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.5, 0.9999, inliers);
        
    list<std::shared_ptr<cv::KeyPoint>>::iterator itr = prev->getPointfeatures().begin();

    // list is cleared first
    if (m_list_keypoints.size() != 0)
    {
        m_list_keypoints.clear();
    }
    int tracked_cnt = 0;
    while (itr != prev->getPointfeatures().end())    
    {

        if (inliers[tracked_cnt] == 1 && status[tracked_cnt] == 1)        
        {
            if((*itr)->history == 1){
                // triangulation

            }
            // only this is valid so connect
            std::shared_ptr<cv::KeyPoint> ptrkpt(new cv::KeyPoint());
            ptrkpt->frameid = getID();
            ptrkpt->global_id = (*itr)->global_id;
            ptrkpt->pt = pts2[tracked_cnt];
            // this is shared_ptr, so basically it is the same.
            ptrkpt->landmark = (*itr)->landmark;
            ptrkpt->color = (*itr)->color;
            ptrkpt->addedtograph = (*itr)->addedtograph;
            // this is shared_ptr, so basically it is the same.
            // *itr is prev
            (*itr)->ptrNext = ptrkpt;
            ptrkpt->ptrPrev = (*itr);
            ptrkpt->history = (*itr)->history + 1;
            m_list_keypoints.push_back(ptrkpt);
            ++itr;
            ++tracked_cnt;
        }
        else
        {
            // remove this feature from the list
            prev->getPointfeatures().erase(itr++);
            tracked_cnt++;
        }        
    }
    if (m_list_keypoints.size() < params.m_stParam.pF.m_s32MinTrackedFeatureNumbers)
    {
        extractGFTTFeaturePoints(params);
    }

    if (m_list_keypoints.size() < params.m_stParam.pF.m_s32MinTrackedFeatureNumbers)
    {
        return false;
    }

    return true;
}

bool Frame::doMatch(const Parameters &params, std::shared_ptr<Frame> prev)
{
    // if (m_vector_featurepoints.size() == 0)
    // {
    // }
}

std::list<std::shared_ptr<cv::KeyPoint>> &Frame::getPointfeatures()
{
    return m_list_keypoints;
}

Preintegrator& Frame::getPreintegrator(){
    return m_preintegrator_prev2curr;
}