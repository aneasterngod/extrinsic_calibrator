#include "Calibrator.h"

Calibrator::Calibrator()
{
    m_b_online = false;
    m_int_steps = 1;
    for (int i = 0; i < 6; i++)
    {
        // it's like shift left by #
        //m_lowpassfilters[i].setCutOffFrequency(5); // after 200(1000/5) is valid (warm up stage)
        //m_lowpassfilters[i].setCutOffFrequency(10); // after 100(1000/10) is valid (warm up stage)
        m_lowpassfilters[i].setCutOffFrequency(20); // after 50(1000/20) is valid (warm up stage) 50frame is about 0.02sec
        m_lowpassfilters[i].setDeltaTime(0.001);
    }
}

Calibrator::~Calibrator()
{
}

void Calibrator::init(std::shared_ptr<GlobalParams> p)
{
    m_shared_ptr_globalparams = p;
    if (FILEPATH != "")
    {
        // this is reading logging
        // almost single threaded behavior
        m_b_online = false;
        readImageInfo(FILEPATH);
        readImuData(FILEPATH);
    }
    else
    {
        // this is online
        m_b_online = true;
    }
}

void Calibrator::run()
{
    if (m_b_online)
    {
        m_thread_filefeeder = std::thread(&Calibrator::online_feeder, this);
    }
    else
    {
        m_thread_filefeeder = std::thread(&Calibrator::file_feeder, this);
    }
    m_thread_filefeeder.join();
}

void Calibrator::readImageInfo(string path)
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

void Calibrator::readImuData(string path)
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

void Calibrator::file_feeder()
{
    cout << "File Feeder" << endl;
    //    ofstream ofs("test.txt");
    //    ofstream ofs2("test2.txt");
    int s32_imu_count = 0;
    bool b_first = true;
    long id = 0;
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
            //            ofs << imudata.getTimestamp() << " " << imudata.getAcc().transpose() << " " << imudata.getGyro().transpose() << endl;
            //            ofs2 << filteredimudata.getTimestamp() << " " << filteredimudata.getAcc().transpose() << " " << filteredimudata.getGyro().transpose() << endl;
            ar_imudata.push_back(filteredimudata);
            m_deque_imudata.pop_front();
            s32_imu_count++;
        }
        std::shared_ptr<FrameData> framedata(new FrameData(m_shared_ptr_globalparams));
        framedata->setID(id);
        id++;
        m_deque_imagedata.pop_front();

        framedata->setImageData(imagedata);
        framedata->setImuData(ar_imudata);
        if (s32_imu_count > 200)
        {
            if (b_first)
            {
                b_first = false;
                framedata->clearImuData();
            }
            // m_deque_disposable_framedata.push_back(framedata);
            doProcess(framedata);
            // if (createKeyframe(framedata))
            // {
            //     m_vector_shared_ptr_keyframes.push_back(framedata);
            // }
        }
    }
    // ofs.close();
    // ofs2.close();
}

void Calibrator::online_feeder()
{
    // it's more complicated, since the data is constantly obtained.
    while (1)
    {
    }
}

bool Calibrator::createKeyframe(std::shared_ptr<FrameData> currframe)
{
    if (m_vector_shared_ptr_keyframes.size() == 0)
        return true;
    Sophus::SE3d diffpose = m_vector_shared_ptr_keyframes.back()->getPose().inverse() * currframe->getPose();
    double transdiff = diffpose.translation().norm();
    double rotxdiff = diffpose.angleX() * rad2deg;
    double rotydiff = diffpose.angleY() * rad2deg;
    double rotzdiff = diffpose.angleZ() * rad2deg;
    if (transdiff > 1)
    {
        return true;
    }
    if (rotxdiff > 5)
    {
        return true;
    }
    if (rotydiff > 5)
    {
        return true;
    }
    if (rotzdiff > 5)
    {
        return true;
    }
    return false;
}

void Calibrator::doProcess(std::shared_ptr<FrameData> fd)
{
    std::shared_ptr<FrameData> currframe = fd;
    m_deque_longterm_all_framedata.push_back(currframe);

    // always first time only taken the image into account.
    if (m_shared_ptr_globalparams->getMatchingType() == MATCHING)
    {
    }
    else if (m_shared_ptr_globalparams->getMatchingType() == TRACKING)
    {
        if (!m_b_online)
        {
            // load image
            currframe->getImageData().loadImage(currframe->getImageData().getImgfliepath());
            currframe->getImageData().undistort(m_shared_ptr_globalparams->getKcv(), m_shared_ptr_globalparams->getDcv());
        }
        if (fd->getImuData().size() == 0)
        {
            // this is the first frame, so create keyframe
            createKF(currframe);
            m_sharedptr_latest_trackedframe = currframe;
        }
        else
        {
            // feature tracking
            if (doTrack(m_sharedptr_latest_trackedframe, currframe))
            {
                cout << "Success tracking " << currframe->getID() << " "
                     << "on " << m_sharedptr_latest_trackedframe->getID() << endl;
                m_sharedptr_latest_trackedframe = currframe;
                if (doBA(currframe))
                {
                }
            }
            else
            {
                cout << "Fail tracking " << currframe->getID() << " "
                     << "on " << m_sharedptr_latest_trackedframe->getID() << endl;
                createKF(m_sharedptr_latest_trackedframe);
            }

            // if (currframe->doTracking(m_sharedptr_latest_trackedframe))
            // {
            //     m_sharedptr_latest_trackedframe = currframe;
            //     // tracking is successful, then pose estimate
            //     // BA

            //     // m_deque_longterm_tracked_framedata.push_back(currframe);
            //     // generatePreintegrator(currframe);
            //     // if(doBA(m_deque_longterm_poseestimated_framedata,currframe)){
            //     //     m_deque_longterm_poseestimated_framedata.push_back(currframe);
            //     // }else{
            //     //     // back to the generatePreintegrator

            //     // }
            // }
            // else
            // {
            //     // create keyframe
            //     createKF(currframe);
            // }
            // generate preintegrator
            // // from previously generated and current one
            // generatePreintegrator(m_deque_disposable_framedata, m_vector_processed_framedata, fd);
            // // get position

            // if (doBA(m_vector_processed_framedata, fd))
            // {

            //     cv::Mat disp = fd->getImageData().getUndistortedImage().clone();
            //     cv::cvtColor(disp, disp, cv::COLOR_GRAY2BGR);
            //     m_cvvisualizer.addPointFeatures(disp, fd->getPointFeatures());
            //     cv::imshow("test", disp);
            //     //cv::imshow("distroted", fd->getImageData().getImage());
            //     cv::waitKey(-1);
            // }
            // else
            // {
            // }
        }
    }
}

bool Calibrator::doBA(std::shared_ptr<FrameData> currframe)
{
    // m_sharedptr_ceresproblem_for_ba
    ceres::Solver::Options options;
    //	options.linear_solver_type = ceres::SPARSE_SCHUR;
    //	options.linear_solver_type = ceres::DENSE_SCHUR;
    //	options.linear_solver_type = ceres::DENSE_QR;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    //	options.max_num_iterations = 20;
    //	options.num_linear_solver_threads = 24;
    //	options.function_tolerance = 1.0e-32;
    //	options.min_trust_region_radius = 1.0e-50;
    //	options.max_solver_time_in_seconds = 0.2;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    for(int i=0;i<currframe->getPointFeatures().size();i++){
        Eigen::Matrix3d K = m_shared_ptr_globalparams->getK();
        Eigen::Vector3d ipt(currframe->getPointFeatures()[i]->pt.x,currframe->getPointFeatures()[i]->pt.y,1);
        CostFunction *cost_function = new ceres::NumericDiffCostFunction<ceres_BA, ceres::CENTRAL, 3, 6, 3>(
            new ceres_BA(ipt, K));
        //m_sharedptr_ceresproblem_for_ba->AddResidualBlock(cost_function, NULL, currframe->getRawPose(), currframe->getPointFeatures()[i]->sharedptr_3dpt);
    }
    

    Solve(options, m_sharedptr_ceresproblem_for_ba.get(), &summary);
}

bool Calibrator::doTrack(std::shared_ptr<FrameData> prevframe, std::shared_ptr<FrameData> currframe)
{
    vector<uchar> status;
    vector<float> errors;
    vector<cv::Point2f> pts1;
    vector<cv::Point2f> pts2;
    FrameData::keypoint2point(prevframe->getPointFeatures(), pts1);
    pts2.resize(pts1.size());
    cv::Size winSize = cv::Size(21, 21);
    int maxLevel = 3;
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.0001);
    int flags = 0;
    double minEigThreshold = m_shared_ptr_globalparams->getmineigen();
    cv::calcOpticalFlowPyrLK(prevframe->getImageData().getUndistortedImage(), currframe->getImageData().getUndistortedImage(), pts1, pts2, status, errors, winSize, maxLevel, criteria, flags,
                             minEigThreshold);
    vector<std::shared_ptr<cv::KeyPoint>> dst;
    FrameData::point2keypoint(pts2, prevframe->getPointFeatures(), dst);
    currframe->setPointFeatures(dst);
    int tracked_cnt = 0;
    for (int i = 0; i < currframe->getPointFeatures().size(); i++)
    {
        if (status[i] == 1)
        {
            if (prevframe->getPointFeatures()[i]->valid)
            {
                currframe->getPointFeatures()[i]->valid = true;
                tracked_cnt++;
            }
        }

        else
        {
            currframe->getPointFeatures()[i]->valid = false;
        }
        currframe->getPointFeatures()[i]->global_id = prevframe->getPointFeatures()[i]->global_id;
        currframe->getPointFeatures()[i]->sharedptr_3dpt = prevframe->getPointFeatures()[i]->sharedptr_3dpt;
        currframe->getPointFeatures()[i]->sharedptr_prev = prevframe->getPointFeatures()[i];
        prevframe->getPointFeatures()[i]->sharedptr_next = currframe->getPointFeatures()[i];
    }
    cout << "Tracked feature number: " << tracked_cnt << endl;

    if (tracked_cnt < m_shared_ptr_globalparams->getMinimumMaintainedTrackedFeatureNumber())
    {
        // //computeFastFeature(true);
        // bool accum = true;
        // bool gen3dpt = true;
        // currframe->computeGFTFeature(accum, gen3dpt);
        // cout << "Feature added and new accumulated feature number: " << currframe->getPointFeatures().size() << " on " << currframe->getID() << endl;
        return false;
    }
    // if(currframe->getPointFeatures().size() < m_shared_ptr_globalparams->getMinimumMaintainedTrackedFeatureNumber())
    //     return false;
    return true;
}

void Calibrator::generatePreintegrator(std::shared_ptr<FrameData> currframe)
{
    // cout << currframe->getImageData().getTimestamp() << endl;
    // if (!m_deque_longterm_poseestimated_framedata.empty())
    // {
    //     currframe->getPreintegrator()->setPrevOmega(m_deque_longterm_poseestimated_framedata.back()->getPreintegrator()->getCurrOmega());
    //     currframe->getPreintegrator()->setLatestTS(m_deque_longterm_poseestimated_framedata.back()->getPreintegrator()->getLatestTS());
    // }
    // while (1)
    // {
    //     if (m_deque_disposable_all_framedata.empty())
    //     {
    //         break;
    //     }
    //     for (int i = 0; i < m_deque_disposable_all_framedata.front()->getImuData().size(); i++)
    //     {
    //         cout << "Adding signal: " << m_deque_disposable_all_framedata.front()->getImuData()[i].getAcc().transpose() << " "
    //              << m_deque_disposable_all_framedata.front()->getImuData()[i].getGyro().transpose() << " " << m_deque_disposable_all_framedata.front()->getImuData()[i].getTimestamp() << endl;

    //         currframe->getPreintegrator()->addSignals(m_deque_disposable_all_framedata.front()->getImuData()[i].getAcc()(0),
    //                                                   m_deque_disposable_all_framedata.front()->getImuData()[i].getAcc()(1),
    //                                                   m_deque_disposable_all_framedata.front()->getImuData()[i].getAcc()(2),
    //                                                   m_deque_disposable_all_framedata.front()->getImuData()[i].getGyro()(0),
    //                                                   m_deque_disposable_all_framedata.front()->getImuData()[i].getGyro()(1),
    //                                                   m_deque_disposable_all_framedata.front()->getImuData()[i].getGyro()(2),
    //                                                   m_deque_disposable_all_framedata.front()->getImuData()[i].getTimestamp());
    //     }
    //     m_deque_disposable_all_framedata.pop_front();
    // }
    // currframe->getPreintegrator()->printAll();
}

void Calibrator::generatePreintegrator(deque<std::shared_ptr<FrameData>> &disposable_dequeframes, vector<std::shared_ptr<FrameData>> &processedframes, std::shared_ptr<FrameData> fd)
{
    if (processedframes.size() != 0)
    {

        fd->getPreintegrator()->setPrevOmega(processedframes.back()->getPreintegrator()->getCurrOmega());
        fd->getPreintegrator()->setLatestTS(processedframes.back()->getPreintegrator()->getLatestTS());
    }
    while (1)
    {
        if (disposable_dequeframes.size() == 0)
            break;
        for (int i = 0; i < disposable_dequeframes.front()->getImuData().size(); i++)
        {
            cout << "Adding signal: " << disposable_dequeframes.front()->getImuData()[i].getAcc().transpose() << " " << disposable_dequeframes.front()->getImuData()[i].getGyro().transpose() << " " << disposable_dequeframes.front()->getImuData()[i].getTimestamp() << endl;

            fd->getPreintegrator()->addSignals(disposable_dequeframes.front()->getImuData()[i].getAcc()(0),
                                               disposable_dequeframes.front()->getImuData()[i].getAcc()(1),
                                               disposable_dequeframes.front()->getImuData()[i].getAcc()(2),
                                               disposable_dequeframes.front()->getImuData()[i].getGyro()(0),
                                               disposable_dequeframes.front()->getImuData()[i].getGyro()(1),
                                               disposable_dequeframes.front()->getImuData()[i].getGyro()(2), disposable_dequeframes.front()->getImuData()[i].getTimestamp());
        }
        fd->getPreintegrator()->printAll();
        disposable_dequeframes.pop_front();
    }
}

double Calibrator::moveDistance(std::shared_ptr<FrameData> f1, std::shared_ptr<FrameData> f2)
{
    double sum = 0;
    int cnt = 0;
    for (int i = 0; i < f2->getPointFeatures().size(); i++)
    {
        std::shared_ptr<cv::KeyPoint> cursor = f2->getPointFeatures()[i];
        while (1)
        {
            if (cursor->ptr_connectedframe == f1.get())
            {
                break;
            }
            if (!cursor->sharedptr_prev)
            {
                break;
            }
            cursor = cursor->sharedptr_prev;
        }
        cv::Point2f diff = cursor->pt - f2->getPointFeatures()[i]->pt;
        sum += sqrt(diff.x * diff.x + diff.y * diff.y);
        cnt++;
    }
    sum /= cnt;
    return sum;
}

void Calibrator::filterFundamental(std::shared_ptr<FrameData> f1)
{
    vector<cv::Point2f> point1;
    vector<cv::Point2f> point2;
    FrameData::keypoint2point(f1->getPointFeatures(), point1);
    for (int i = 0; i < point1.size(); i++)
    {
        cv::Point2f pt = f1->getPointFeatures()[i]->sharedptr_next->pt;
        point2.push_back(pt);
    }
    cv::Mat mask;
    vector<uchar> inliers(point1.size(), 0);
    cv::Mat F = findFundamentalMat(point1, point2, cv::FM_RANSAC, 1.0, 0.99, inliers);
    for (int i = 0; i < point1.size(); i++)
    {
        if (!inliers[i])
        {
            f1->getPointFeatures()[i]->valid = false;
            f1->getPointFeatures()[i]->sharedptr_next->valid = false;
        }
    }
}

bool Calibrator::doBA(vector<std::shared_ptr<FrameData>> &processedframes, std::shared_ptr<FrameData> fd)
{
    double dist = moveDistance(processedframes.back(), fd);
    if (dist < 10)
        return false;
    filterFundamental(processedframes.back());
    // for (int i = 0; i < currentinputindices.size(); i++) {
    // 	Eigen::Matrix<double, 6, 1> epose(m_pg.m_data[currentinputindices[i]]->m_pose.get());
    // 	for (int j = 0; j < m_pg.m_data[currentinputindices[i]]->m_matches.size(); j++) {
    // 		int t = m_pg.m_data[currentinputindices[i]]->m_matches[j].trainIdx;
    // 		long ti = m_pg.m_data[currentinputindices[i]]->m_img->m_tracklet_ids[t];
    // 		int q = m_pg.m_data[currentinputindices[i]]->m_keyframe->m_img->m_trackletid2index[ti];
    // 		cv::Point2f currpt = m_pg.m_data[currentinputindices[i]]->m_img->m_keypoints[t].pt;
    // 		Eigen::Vector3d ecurrpt;
    // 		ecurrpt << currpt.x, currpt.y, 1;
    // 		if (is3dptvalid(m_pg.m_data[currentinputindices[i]]->m_keyframe->m_map->m_worldpts[q].m_pt.get())) {
    // 			CostFunction* cost_function = new NumericDiffCostFunction<ceres_REFINER, ceres::CENTRAL, 3, 3>(new ceres_REFINER(ecurrpt, epose, m_K));
    // 			m_refiner_problem->AddResidualBlock(cost_function, NULL, m_pg.m_data[currentinputindices[i]]->m_keyframe->m_map->m_worldpts[q].m_pt.get());
    // 			cnt++;
    // 		}
    // 	}
    // 	cout << "[refinemap_loop] refiner input: " << m_pg.m_data[currentinputindices[i]]->m_frameid << endl;
    // }
    return true;
}

void Calibrator::createKF(std::shared_ptr<FrameData> kf)
{
    bool accum = true; // accum은 이미 피쳐포인트가 있지만 또 추가할 경우 accum을 넣는다.
    bool gen3dpt = true;
    kf->computeGFTFeature(accum, gen3dpt);
    cout << "Feature added and new accumulated feature number: " << kf->getPointFeatures().size() << " on " << kf->getID() << endl;
    m_sharedptr_ceresproblem_for_ba = std::shared_ptr<ceres::Problem>(new ceres::Problem);
    m_deque_longterm_keyframe_framedata.push_back(kf);

    cv::Mat disp = kf->getImageData().getUndistortedImage().clone();
    cv::cvtColor(disp, disp, cv::COLOR_GRAY2BGR);
    m_cvvisualizer.addPointFeatures(disp, kf->getPointFeatures());
    cv::imshow("keyframe", disp);
    //cv::imshow("distroted", fd->getImageData().getImage());
    cv::waitKey(-1);
}