#ifndef CvVisualizer_H_
#define CvVisualizer_H_

#include <opencv2/opencv.hpp>
#include <vector>


class CvVisualizer{
public:
    CvVisualizer();
    ~CvVisualizer();
    void addPointFeatures(cv::Mat& img, const std::vector<cv::KeyPoint>& kpts);
private:
    std::vector<uint8_t> m_vector_u8_colormapR;
    std::vector<uint8_t> m_vector_u8_colormapG;
    std::vector<uint8_t> m_vector_u8_colormapB;
};


#endif