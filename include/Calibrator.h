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
#include "ceres_BA.h"


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
        bool createKeyframe(std::shared_ptr<FrameData> currframe);
        void doProcess(std::shared_ptr<FrameData> fd);
        bool doBA(std::shared_ptr<FrameData> currframe);
        bool doTrack(std::shared_ptr<FrameData> prevframe, std::shared_ptr<FrameData> currframe);        
        void generatePreintegrator(std::shared_ptr<FrameData> currframe);
        void generatePreintegrator(deque<std::shared_ptr<FrameData>> &disposable_dequeframes, vector<std::shared_ptr<FrameData>>& processedframes, std::shared_ptr<FrameData> fd);
        double moveDistance(std::shared_ptr<FrameData> f1, std::shared_ptr<FrameData> f2 );
        void filterFundamental(std::shared_ptr<FrameData> f1);
        bool doBA(vector<std::shared_ptr<FrameData>> &processedframes, std::shared_ptr<FrameData> fd);
        void createKF(std::shared_ptr<FrameData> kf);
    private:
        bool m_b_online;
        std::thread m_thread_filefeeder;


        deque<ImageData> m_deque_imagedata;        
        deque<ImuData> m_deque_imudata;
        deque<std::shared_ptr<FrameData>> m_deque_longterm_all_framedata;
        deque<std::shared_ptr<FrameData>> m_deque_longterm_keyframe_framedata;
        std::shared_ptr<FrameData> m_sharedptr_latest_trackedframe;
        std::shared_ptr<ceres::Problem> m_sharedptr_ceresproblem_for_ba;
        int m_int_steps;
        CvVisualizer m_cvvisualizer;

        LowPassFilter m_lowpassfilters[6];
        std::shared_ptr<GlobalParams> m_shared_ptr_globalparams;
        vector<std::shared_ptr<FrameData> > m_vector_shared_ptr_keyframes;
};


#endif