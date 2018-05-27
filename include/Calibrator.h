#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "excalib_common.h"
#include "ImageData.h"
#include "ImuData.h"
#include "FrameData.h"

class Calibrator{
    public:
        Calibrator();
        ~Calibrator();
        void init();
        void run();
        void readImageInfo(string path);
        void readImuData(string path);
        void file_feeder();
        void online_feeder();
        void doProcess(excalib::FrameData& fd);
    private:
        bool m_b_online;
        deque<excalib::ImageData> m_ar_imagedata;
        deque<excalib::ImuData> m_ar_imudata;
        std::thread m_thread_filefeeder;

};


#endif