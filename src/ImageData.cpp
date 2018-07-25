#include "ImageData.h"

ImageData::ImageData()
{
}

ImageData::~ImageData()
{
}
ImageData &ImageData::operator=(const ImageData &dat)
{
    m_s64_timestamp = dat.m_s64_timestamp;
    m_str_filepath = dat.m_str_filepath;
    m_cvmat_img = dat.m_cvmat_img.clone();
    return *this;
}
ImageData::ImageData(const ImageData &dat){
    m_s64_timestamp = dat.m_s64_timestamp;
    m_str_filepath = dat.m_str_filepath;
    m_cvmat_img = dat.m_cvmat_img.clone();
}

void ImageData::setTimestamp(int64_t ts)
{
    m_s64_timestamp = ts;
}

int64_t ImageData::getTimestamp()
{
    return m_s64_timestamp;
}

void ImageData::setImgfilepath(std::string path)
{
    m_str_filepath = path;
}

std::string ImageData::getImgfliepath()
{
    return m_str_filepath;
}

void ImageData::loadImage(std::string path){
    m_cvmat_img = cv::imread(path, cv::IMREAD_GRAYSCALE);
}

void ImageData::undistort(const cv::Mat& K, const cv::Mat& D){
    // this is unrectified image, so rectify the image
    cv::undistort(m_cvmat_img, m_cvmat_undistorted_img, K, D);    
}

const cv::Mat& ImageData::getImage(){
    return m_cvmat_img;
}
const cv::Mat& ImageData::getUndistortedImage(){
    return m_cvmat_undistorted_img;
}
void ImageData::print(){
     if(!m_cvmat_img.empty()){
        cout << "ts: " << m_s64_timestamp << " width: " << m_cvmat_img.rows << " height: " << m_cvmat_img.cols << " ";
        
     }
     else{
         cout << "ts: " << m_s64_timestamp << " ";
    }
    if(m_str_filepath != "") {
        cout << "Filepath: " << m_str_filepath << endl;
    }
}
