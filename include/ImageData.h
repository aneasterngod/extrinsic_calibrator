#ifndef EXCALIBIMAGEDATA_H_
#define EXCALIBIMAGEDATA_H_

#include "excalib_common.h"

namespace excalib{

class ImageData{
    public:
        ImageData();
        ~ImageData();
        ImageData & operator=(const ImageData &dat);
        ImageData(const ImageData &dat);
        void setTimestamp(int64_t ts);
        int64_t getTimestamp();
        void setImgfilepath(std::string path);
        std::string getImgfliepath();
        void print();
    private:
        std::string m_str_filepath;
        cv::Mat m_cvmat_img;
        int64_t m_s64_timestamp;
};

}

#endif