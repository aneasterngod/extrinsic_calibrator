#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "excalib_common.h"
#include "ImageData.h"
#include "ImuData.h"
#include "FrameData.h"
#include "Preintegrator.h"
#include "RightJacobian.h"
#include "LowPassFilter.h"
#include "GlobalParams.h"
#include "CvVisualizer.h"
#include "worldpt.h"

class Calibrator{
    public:
        Calibrator();
        ~Calibrator();
        void init(std::shared_ptr<GlobalParams> p);
        void run();
        void readImageInfo(string path);
        void readImuData(string path);
        void file_feeder();
        void online_feeder();
        bool createKeyframe(std::shared_ptr<excalib::FrameData> currframe);
        void doProcess(std::shared_ptr<excalib::FrameData> fd);
        void generatePreintegrator(deque<std::shared_ptr<excalib::FrameData>> &disposable_dequeframes, vector<std::shared_ptr<excalib::FrameData>>& processedframes, std::shared_ptr<excalib::FrameData> fd);
        double moveDistance(std::shared_ptr<excalib::FrameData> fd);
        void doBA(vector<std::shared_ptr<excalib::FrameData>> &processedframes, std::shared_ptr<excalib::FrameData> fd);
    private:
        bool m_b_online;
        std::thread m_thread_filefeeder;


        deque<excalib::ImageData> m_deque_imagedata;
        deque<excalib::ImuData> m_deque_imudata;
        deque<std::shared_ptr<excalib::FrameData>> m_deque_disposable_framedata;
        vector<std::shared_ptr<excalib::FrameData>> m_vector_processed_framedata;
        int m_int_steps;
        CvVisualizer m_cvvisualizer;

        LowPassFilter m_lowpassfilters[6];
        std::shared_ptr<GlobalParams> m_shared_ptr_globalparams;
        vector<std::shared_ptr<excalib::FrameData> > m_vector_shared_ptr_keyframes;
};


#endif