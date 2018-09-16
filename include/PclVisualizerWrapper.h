#ifndef PCLVISUALIZERWRAPPER_H_
#define PCLVISUALIZERWRAPPER_H_

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <thread>

using namespace std;

class PclVisualizerWrapper{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    PclVisualizerWrapper();
    ~PclVisualizerWrapper();
    void setFeaturePoints(const vector<std::shared_ptr<cv::KeyPoint> >& dat);
    void run();
    void main_thread();
private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_curr_cloud;
    std::thread m_main_thread;
};

#endif