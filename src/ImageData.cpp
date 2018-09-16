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
    m_cvmat_undistorted_img = dat.m_cvmat_undistorted_img.clone();
    return *this;
}

ImageData::ImageData(const ImageData &dat){
    m_s64_timestamp = dat.m_s64_timestamp;
    m_str_filepath = dat.m_str_filepath;
    m_cvmat_img = dat.m_cvmat_img.clone();
    m_cvmat_undistorted_img = dat.m_cvmat_undistorted_img.clone();
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

string ImageData::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}