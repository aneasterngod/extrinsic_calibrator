#include "PclVisualizerWrapper.h"

PclVisualizerWrapper::PclVisualizerWrapper()
{
}

PclVisualizerWrapper::~PclVisualizerWrapper()
{
}

void PclVisualizerWrapper::setFeaturePoints(const vector<std::shared_ptr<cv::KeyPoint>> &dat)
{
    m_curr_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < dat.size(); i++)
    {
        if (dat[i]->valid)
        {
            pcl::PointXYZRGB pt;
            pt.x = dat[i]->sharedptr_3dpt.get()[0];
            pt.y = dat[i]->sharedptr_3dpt.get()[1];
            pt.z = dat[i]->sharedptr_3dpt.get()[2];
            uint8_t r = dat[i]->color(2), g = dat[i]->color(1), b = dat[i]->color(0);
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pt.rgb = *reinterpret_cast<float *>(&rgb);
            m_curr_cloud->push_back(pt);
        }
    }
    m_curr_cloud->width = (int)m_curr_cloud->points.size();
    m_curr_cloud->height = 1;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_curr_cloud);
    m_viewer->removeAllPointClouds();
    m_viewer->addPointCloud<pcl::PointXYZRGB>(m_curr_cloud, rgb, "sample cloud");
}

void PclVisualizerWrapper::run()
{
    m_main_thread = std::thread(&PclVisualizerWrapper::main_thread, this);
}

void PclVisualizerWrapper::main_thread()
{
    m_viewer = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
    m_viewer->setBackgroundColor(0, 0, 0);
    m_viewer->addCoordinateSystem(1.0);
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    m_viewer->initCameraParameters();
    while (!m_viewer->wasStopped())
    {
        m_viewer->spinOnce();
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
