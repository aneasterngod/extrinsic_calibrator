#ifndef ExtCalibrator_H_
#define ExtCalibrator_H_

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <thread>

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <deque>

#include "LowPassFilter.h"
#include "Parameters.h"
#include "ImuData.h"
#include "ImageData.h"
#include "Frame.h"

class ExtCalibrator
{
  public:
    ExtCalibrator(const Parameters &param);
    ~ExtCalibrator();
    void init();
    void run();
    void preintegrates(std::shared_ptr<Frame> startframe, std::shared_ptr<Frame> endframe);
    void doIsam(std::shared_ptr<Frame> currframe);
    double randomsign(double r);
    double random(double lower, double upper);
    void viewer_thread();
    void readImageInfo(string path);
    void readImuData(string path);
    void createKeyframe(const std::shared_ptr<Frame> &frame);
    void displayFeatures(const cv::Mat& img, const std::list<std::shared_ptr<cv::KeyPoint>> &kpts);
    double moveDistance(std::shared_ptr<Frame> f1, std::shared_ptr<Frame> f2);
  private:
    
    Parameters m_param;
    LowPassFilter m_lowpassfilters[6];

    std::deque<ImageData> m_deque_imagedata;
    std::deque<ImuData> m_deque_imudata;
    std::shared_ptr<Frame> m_ptrPrevTrackedframe;
    std::shared_ptr<Frame> m_ptrPrevframe;

    bool m_bVeryFirst;
    gtsam::NonlinearFactorGraph m_graph;
    gtsam::Cal3_S2::shared_ptr m_ptrK;
    gtsam::noiseModel::Isotropic::shared_ptr m_ptrMeasurementNoise;
    gtsam::noiseModel::Diagonal::shared_ptr m_ptrPoseNoise;
    gtsam::noiseModel::Isotropic::shared_ptr m_ptrPointNoise;
    gtsam::Values m_initialEstimates;
    gtsam::Values m_currentEstimates;
    std::shared_ptr<gtsam::ISAM2> m_ptrISAM2;
    gtsam::ISAM2Params m_isam2param;

    std::shared_ptr<pcl::visualization::PCLVisualizer> m_ptrPCLviewer;
    std::thread m_thread_viewer;

    std::vector<uint8_t> m_vector_u8_colormapR;
    std::vector<uint8_t> m_vector_u8_colormapG;
    std::vector<uint8_t> m_vector_u8_colormapB;

};

#endif