#include "ExtCalibrator.h"

ExtCalibrator::ExtCalibrator(const Parameters &param)
{
    m_param = param;

    for (int i = 0; i < 10000; i++)
    {
        float r = (float)rand() / (float)RAND_MAX;
        float g = (float)rand() / (float)RAND_MAX;
        float b = (float)rand() / (float)RAND_MAX;
        r = r * 255;
        g = g * 255;
        b = b * 255;
        m_vector_u8_colormapR.push_back((uint8_t)r);
        m_vector_u8_colormapG.push_back((uint8_t)g);
        m_vector_u8_colormapB.push_back((uint8_t)b);
    }
    m_bVeryFirst = true;
}

ExtCalibrator::~ExtCalibrator()
{
}

void ExtCalibrator::init()
{
    m_thread_viewer = std::thread(&ExtCalibrator::viewer_thread, this);
    for (int i = 0; i < 6; i++)
    {
        // it's like shift left by #
        m_lowpassfilters[i].setCutOffFrequency(m_param.m_stParam.pL.m_f32Freq); // after 50(1000/20) is valid (warm up stage) 50frame is about 0.02sec
        m_lowpassfilters[i].setDeltaTime(m_param.m_stParam.pL.m_f32DeltaT);
    }

    readImageInfo(m_param.m_stParam.m_str_logfilepath);
    readImuData(m_param.m_stParam.m_str_logfilepath);
}

long __GFRAMEID__ = 0;

void ExtCalibrator::run()
{
    int s32_imu_count = 0;
    bool b_first = true;
    // Define the camera calibration parameters
    m_ptrK = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(m_param.m_stParam.pC.m_mat3_K(0, 0), m_param.m_stParam.pC.m_mat3_K(1, 1), 0.0, m_param.m_stParam.pC.m_mat3_K(0, 2), m_param.m_stParam.pC.m_mat3_K(1, 2)));
    // Define the camera observation noise model
    m_ptrMeasurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);                                                                                 // one pixel in u and v
    m_ptrPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    m_ptrPointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    m_isam2param.relinearizeThreshold = 0.01;
    m_isam2param.relinearizeSkip = 1;
    m_ptrISAM2 = std::shared_ptr<gtsam::ISAM2>(new gtsam::ISAM2(m_isam2param));
    while (1)
    {
        // read first image and first imu
        ImageData imagedata = m_deque_imagedata.front();
        vector<ImuData> ar_imudata;
        while (1)
        {
            // store imu
            ImuData imudata = m_deque_imudata.front();
            // 여기서 lowpass filtering 후 shift (change timestamp)
            // 50frame 0.07초 쉬프트 하면 맞는지 비교할것 70000 얼추 맞다.
            // octave에서 첫번째 컬럼인 타임스템프 첫번째 인자를 빼서 plot(imu(:,1), imu(:,2)) 이런식으로 비교가능하다.
            // 그리고 200이후부터
            ImuData filteredimudata = imudata;
            filteredimudata.setTimestamp(filteredimudata.getTimestamp() - 70000);
            if (imagedata.getTimestamp() < filteredimudata.getTimestamp())
            {
                break;
            }
            float acc[3];
            for (int i = 0; i < 3; i++)
            {
                acc[i] = m_lowpassfilters[i].update(imudata.getAcc()(i));
            }
            float gyro[3];
            for (int i = 0; i < 3; i++)
            {
                gyro[i] = m_lowpassfilters[i + 3].update(imudata.getGyro()(i));
            }
            filteredimudata.setAcc(acc[0], acc[1], acc[2]);
            filteredimudata.setGyro(gyro[0], gyro[1], gyro[2]);
            cout << "adding imu signal: " << filteredimudata.getTimestamp() << endl;
            ar_imudata.push_back(filteredimudata);
            m_deque_imudata.pop_front();
            s32_imu_count++;
        }

        std::shared_ptr<Frame> currframe(new Frame());
        currframe->setID(__GFRAMEID__);
        __GFRAMEID__++;
        m_deque_imagedata.pop_front();

        currframe->setImageData(imagedata);
        currframe->setImuData(ar_imudata);
        if (s32_imu_count > 200)
        {
            currframe->getImageData().loadImage(currframe->getImageData().getImgfliepath());
            currframe->getImageData().undistort(m_param.m_stParam.pC.m_cvmat_K, m_param.m_stParam.pC.m_cvmat_D);
            if (b_first)
            {
                b_first = false;

                // create Keyframe
                createKeyframe(currframe);
                displayFeatures(currframe->getImageData().getUndistortedImage(), currframe->getPointfeatures());
                // because the first one never would have imudata, so clear them up.
                currframe->clearImuData();

                currframe->m_ptrPrev = m_ptrPrevTrackedframe;
                m_ptrPrevTrackedframe = currframe;
                m_ptrPrevframe = currframe;
            }
            else
            {
                // tracking
                currframe->m_ptrPrev = m_ptrPrevframe;
                m_ptrPrevframe->m_ptrNext = currframe;
                
                if (currframe->doTrack(m_param, m_ptrPrevTrackedframe))
                {
                    double dist = moveDistance(m_ptrPrevTrackedframe, currframe);
                    if (dist > 10)
                    {
                        preintegrates(m_ptrPrevTrackedframe, currframe);
                        currframe->getPreintegrator().printAll();
                        // display tracking result
                        displayFeatures(currframe->getImageData().getUndistortedImage(), currframe->getPointfeatures());
                        doIsam(currframe);
                        m_ptrPrevTrackedframe = currframe;
                    }
                }
                m_ptrPrevframe = currframe;
            }
        }
    }
}

void ExtCalibrator::preintegrates(std::shared_ptr<Frame> startframe, std::shared_ptr<Frame> endframe)
{
    endframe->getPreintegrator().clear();
    if (startframe->getPreintegrator().getNumSignals() != 0)
    {
        endframe->getPreintegrator().setPrevOmega(startframe->getPreintegrator().getCurrOmega());
        endframe->getPreintegrator().setLatestTS(startframe->getPreintegrator().getLatestTS());
    }
    std::shared_ptr<Frame> cursor = startframe->m_ptrNext;
    cout << "preintegration: " << endframe->getID() << " " << endframe->getImageData().getTimestamp() << endl;
    while (1)
    {

        //     if (m_deque_disposable_all_framedata.empty())
        //     {
        //         break;
        //     }
        cout << "cursor: " << cursor->getID() << ": " << cursor->getImageData().getTimestamp() << endl;
        for (int i = 0; i < cursor->getImuData().size(); i++)
        {
            cout << "Adding signal: " << cursor->getImuData()[i].getAcc().transpose() << " "
                 << cursor->getImuData()[i].getGyro().transpose() << " " << cursor->getImuData()[i].getTimestamp() << endl;

            endframe->getPreintegrator().addSignals(cursor->getImuData()[i].getAcc()(0),
                                                    cursor->getImuData()[i].getAcc()(1),
                                                    cursor->getImuData()[i].getAcc()(2),
                                                    cursor->getImuData()[i].getGyro()(0),
                                                    cursor->getImuData()[i].getGyro()(1),
                                                    cursor->getImuData()[i].getGyro()(2),
                                                    cursor->getImuData()[i].getTimestamp());
        }

        if (cursor->getID() == endframe->getID())
        {
            break;
        }
        cursor = cursor->m_ptrNext;
    }
}

void ExtCalibrator::doIsam(std::shared_ptr<Frame> currframe)
{

    if (m_bVeryFirst)    
    {
        // add prior for x0
        m_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('x', currframe->getID()), currframe->getGtsamPose(), m_ptrPoseNoise);
    }

    // add landmarks projected factor for xj and if it is newly added then set initial value on it.
    std::list<std::shared_ptr<cv::KeyPoint>>::iterator itr;
    for (itr = currframe->getPointfeatures().begin(); itr != currframe->getPointfeatures().end(); itr++)
    {
        gtsam::Point2 ipt((*itr)->pt.x, (*itr)->pt.y);
        if ((*itr)->history >= 2)
        {
            m_graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(ipt, m_ptrMeasurementNoise,
                                                                                                                gtsam::Symbol('x', currframe->getID()),
                                                                                                                gtsam::Symbol('l', (*itr)->global_id), m_ptrK);
        }
        if ((*itr)->history == 2)
        {
            // gtsam::SimpleCamera prevcamera(m_ptrPrevTrackedframe->getGtsamPose(), *m_ptrK);
            // gtsam::SimpleCamera camera(currframe->getGtsamPose(), *m_ptrK);
            // // this is the first seen features with 3 times
            // gtsam::Point2 prevprevipt((*itr)->ptrPrev->ptrPrev->pt.x, (*itr)->ptrPrev->ptrPrev->pt.y);
            // gtsam::Point2 previpt((*itr)->ptrPrev->pt.x, (*itr)->ptrPrev->pt.y);
            // m_graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(prevprevipt, m_ptrMeasurementNoise,
            //                                                                                                     gtsam::Symbol('x', m_ptrPrevPrevTrackedframe->getID()),
            //                                                                                                     gtsam::Symbol('l', (*itr)->ptrPrev->ptrPrev->global_id), m_ptrK);
            // m_graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(previpt, m_ptrMeasurementNoise,
            //                                                                                                     gtsam::Symbol('x', m_ptrPrevTrackedframe->getID()),
            //                                                                                                     gtsam::Symbol('l', (*itr)->ptrPrev->global_id), m_ptrK);
            // gtsam::Point3 prevprevwpt = prevprevcamera.backproject(prevprevipt, 1);
            // m_initialEstimates.insert<gtsam::Point3>(gtsam::Symbol('l', (*itr)->ptrPrev->ptrPrev->global_id), prevprevwpt);
        }
    }
    // set initial value of xj + random value
    double r1 = random(0, 1.0 * Deg2rad);
    r1 = randomsign(r1);
    double r2 = random(0, 1.0 * Deg2rad);
    r2 = randomsign(r2);
    double r3 = random(0, 1.0 * Deg2rad);
    r3 = randomsign(r3);
    double t1 = random(0, 0.1);
    t1 = randomsign(t1);
    double t2 = random(0, 0.1);
    t2 = randomsign(r2);
    double t3 = random(0, 0.1);
    t3 = randomsign(t3);
    m_initialEstimates.insert(gtsam::Symbol('x', currframe->getID()),
                              m_ptrPrevTrackedframe->getGtsamPose().compose(gtsam::Pose3(gtsam::Rot3::Rodrigues(r1, r2, r3), gtsam::Point3(t1, t2, t3))));
    m_graph.print();
    m_initialEstimates.print();
    m_ptrISAM2->update(m_graph, m_initialEstimates);
    // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
    // If accuracy is desired at the expense of time, update(*) can be called additional times
    // to perform multiple optimizer iterations every step.
    m_ptrISAM2->update();
    gtsam::Values currentEstimate = m_ptrISAM2->calculateEstimate();
    currentEstimate.print();

    // Clear the factor graph and values for the next iteration
    // previous result will be outside data location so put in to cv::KeyPoint
    m_graph.resize(0);
    m_initialEstimates.clear();
}
double ExtCalibrator::randomsign(double r)
{
    if (rand() % 2 == 0)
    {
        r = -r;
    }
    return r;
}
double ExtCalibrator::random(double lower, double upper)
{
    int multiplier = 1000000;
    int s32upper = upper * multiplier;
    int s32lower = lower * multiplier;
    int randomvalue = (rand() %
                       (s32upper - s32lower + 1)) +
                      s32lower;
    double ret = (double)randomvalue / (double)multiplier;
    return ret;
}

void ExtCalibrator::viewer_thread()
{
    m_ptrPCLviewer = std::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
    m_ptrPCLviewer->setBackgroundColor(0, 0, 0);
    m_ptrPCLviewer->addCoordinateSystem(1.0);
    m_ptrPCLviewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    m_ptrPCLviewer->initCameraParameters();
    while (!m_ptrPCLviewer->wasStopped())
    {
        m_ptrPCLviewer->spinOnce();
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
}

void ExtCalibrator::readImageInfo(string path)
{
    string infopath = "";
    if (path.back() == '/')
    {
        infopath = path + "image.txt";
    }
    else
    {
        infopath = path + "/" + "image.txt";
    }
    ifstream ifstream_infopath(infopath);
    while (1)
    {
        if (ifstream_infopath.eof())
        {
            break;
        }
        int64_t s64_ts;
        string str_imgpath;
        ifstream_infopath >> s64_ts >> str_imgpath;
        ImageData imagedata;
        imagedata.setTimestamp(s64_ts);
        imagedata.setImgfilepath(str_imgpath);
        m_deque_imagedata.push_back(imagedata);
    }
    ifstream_infopath.close();
}

void ExtCalibrator::readImuData(string path)
{
    string infopath = "";
    if (path.back() == '/')
    {
        infopath = path + "imu.txt";
    }
    else
    {
        infopath = path + "/" + "imu.txt";
    }
    ifstream ifstream_infopath(infopath);
    while (1)
    {
        if (ifstream_infopath.eof())
        {
            break;
        }
        int64_t s64_ts;
        float f32_acc[3];
        float f32_gyro[3];
        ifstream_infopath >> s64_ts >> f32_acc[0] >> f32_acc[1] >> f32_acc[2] >> f32_gyro[0] >> f32_gyro[1] >> f32_gyro[2];
        ImuData imudata;
        imudata.setTimestamp(s64_ts);
        imudata.setAcc(f32_acc[0], f32_acc[1], f32_acc[2]);
        imudata.setGyro(f32_gyro[0], f32_gyro[1], f32_gyro[2]);
        m_deque_imudata.push_back(imudata);
    }
    ifstream_infopath.close();
}

void ExtCalibrator::createKeyframe(const std::shared_ptr<Frame> &frame)
{
    // 1. extract features
    frame->extractGFTTFeaturePoints(m_param);
    //frame->extractFASTFeaturePoints(m_param);
    frame->setKeyframe(true);
}

void ExtCalibrator::displayFeatures(const cv::Mat &img, const std::list<std::shared_ptr<cv::KeyPoint>> &kpts)
{
    cv::Mat disp = img.clone();
    cv::cvtColor(disp, disp, cv::COLOR_GRAY2BGR);
    std::list<std::shared_ptr<cv::KeyPoint>>::const_iterator itr;
    for (itr = kpts.begin(); itr != kpts.end(); itr++)
    {
        // if (kpts[i].valid)
        cv::circle(disp, (*itr)->pt, 2,
                   cv::Scalar(m_vector_u8_colormapB[(*itr)->global_id], m_vector_u8_colormapG[(*itr)->global_id], m_vector_u8_colormapR[(*itr)->global_id]),
                   2, 8, 0);
    }
    cv::imshow("features", disp);
    cv::waitKey(3);
}

double ExtCalibrator::moveDistance(std::shared_ptr<Frame> f1, std::shared_ptr<Frame> f2)
{
    double sum = 0;
    int cnt = 0;

    std::list<std::shared_ptr<cv::KeyPoint>>::iterator itr;
    for (itr = f2->getPointfeatures().begin(); itr != f2->getPointfeatures().end(); itr++)
    {
        int frameid = f2->getID();
        std::shared_ptr<cv::KeyPoint> cursor = (*itr);
        while (1)
        {
            if (frameid == f1->getID())
            {
                break;
            }
            if (!cursor->ptrPrev)
            {
                break;
            }
            cursor = cursor->ptrPrev;
            frameid--;
        }
        cv::Point2f diff = cursor->pt - (*itr)->pt;
        sum += sqrt(diff.x * diff.x + diff.y * diff.y);
        cnt++;
    }
    sum /= cnt;
    return sum;
}