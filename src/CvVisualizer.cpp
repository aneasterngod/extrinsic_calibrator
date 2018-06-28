#include "CvVisualizer.h"


using namespace std;
CvVisualizer::CvVisualizer()
{
    for (int i = 0; i < 10000; i++)
    {
        float r = (float)rand() / (float)RAND_MAX;
        float g = (float)rand() / (float)RAND_MAX;
        float b = (float)rand() / (float)RAND_MAX;
        r = r * 255;
        g = g * 255;
        b = b * 255;
        m_vector_u8_colormapR.push_back((uint8_t)r);
        m_vector_u8_colormapG.push_back((uint8_t)g);
        m_vector_u8_colormapB.push_back((uint8_t)b);
    }
}

CvVisualizer::~CvVisualizer()
{
}

void CvVisualizer::addPointFeatures(cv::Mat &img, const std::vector<std::shared_ptr<cv::KeyPoint>> &kpts)
{
    for (int i = 0; i < kpts.size(); i++)
    {
        //cv::drawMarker(img, kpts[i].pt, cv::Scalar(m_vector_u8_colormapB[kpts[i].class_id], m_vector_u8_colormapG[kpts[i].class_id], m_vector_u8_colormapR[kpts[i].class_id]), cv::MARKER_DIAMOND, 1, 1, 8);
        if(kpts[i]->valid)
            cv::circle(img, kpts[i]->pt, 2, cv::Scalar(m_vector_u8_colormapB[kpts[i]->global_id], m_vector_u8_colormapG[kpts[i]->global_id], m_vector_u8_colormapR[kpts[i]->global_id]), 2, 8, 0);
    }
}
