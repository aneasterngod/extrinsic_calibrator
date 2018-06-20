#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "excalib_common.h"
#include "ImageData.h"
#include "ImuData.h"
#include "FrameData.h"
#include "Preintegrator2.h"
#include "RightJacobian.h"
#include "LowPassFilter.h"
#include "GlobalParams.h"
#include "CvVisualizer.h"

class Calibrator{
    public:
        Calibrator();
        ~Calibrator();
        void init(std::shared_ptr<GlobalParams>& p);
        void run();
        void readImageInfo(string path);
        void readImuData(string path);
        void file_feeder();
        void online_feeder();
        void doProcess(std::shared_ptr<excalib::FrameData> fd);
        void generatePreintegrator(deque<excalib::FrameData>& queue, int steps, Preintegrator2& preint);
    private:
        bool m_b_online;
        deque<excalib::ImageData> m_deque_imagedata;
        deque<excalib::ImuData> m_deque_imudata;
        std::thread m_thread_filefeeder;
        deque<std::shared_ptr<excalib::FrameData>> m_deque_framedata;
        std::shared_ptr<excalib::FrameData> m_shared_ptr_lastprocessed_framedata;
        int m_int_steps;
        CvVisualizer m_cvvisualizer;

        LowPassFilter m_lowpassfilters[6];
        std::shared_ptr<GlobalParams> m_shared_ptr_globalparams;
};


#endif