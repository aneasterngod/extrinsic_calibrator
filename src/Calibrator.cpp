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

void Calibrator::init()
{
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
        excalib::ImageData imagedata;
        imagedata.setTimestamp(s64_ts);
        imagedata.setImgfilepath(str_imgpath);
        m_ar_imagedata.push_back(imagedata);
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
        excalib::ImuData imudata;
        imudata.setTimestamp(s64_ts);
        imudata.setAcc(f32_acc[0], f32_acc[1], f32_acc[2]);
        imudata.setGyro(f32_gyro[0], f32_gyro[1], f32_gyro[2]);
        m_ar_imudata.push_back(imudata);
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
    while (1)
    {
        // read first image and first imu
        excalib::ImageData imagedata = m_ar_imagedata.front();
        vector<excalib::ImuData> ar_imudata;
        while (1)
        {
            // store imu
            excalib::ImuData imudata = m_ar_imudata.front();
            // 여기서 lowpass filtering 후 shift (change timestamp)
            // 50frame 0.07초 쉬프트 하면 맞는지 비교할것 70000 얼추 맞다.
            // octave에서 첫번째 컬럼인 타임스템프 첫번째 인자를 빼서 plot(imu(:,1), imu(:,2)) 이런식으로 비교가능하다.
            // 그리고 200이후부터
            excalib::ImuData filteredimudata = imudata;
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
            m_ar_imudata.pop_front();
            s32_imu_count++;
        }
        excalib::FrameData framedata;
        m_ar_imagedata.pop_front();

        framedata.setImageData(imagedata);
        framedata.setArImuData(ar_imudata);
        if (s32_imu_count > 200)
        {
            if (b_first)
            {
                b_first = false;
                framedata.clearArImuData();
            }
            doProcess(framedata);
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

void Calibrator::doProcess(excalib::FrameData &fd)
{
    // always first time only taken the image into account.
    if(fd.getArImuData().size()==0){
        // first one 
        // set pose to zero
        // extract feature
        if(!m_b_online){
            // load image
            fd.getImageData().loadImage(fd.getImageData().getImgfliepath());
        }
        fd.computeFastFeature();
    }
    else{

    }
    // if (fd.getArImuData().size() != 0)
    // {
        // if (m_b_firsttimefeed)
        // {
        //     m_b_firsttimefeed = false;
        //     return;
        // }
        // m_ar_framedata.push_back(fd);
        // if (m_ar_framedata.size() >= m_int_steps)
        // {

        //     Preintegrator2 preint;
        //     generatePreintegrator(m_ar_framedata, m_int_steps, preint);
        // } 
        //
    // }
}

void Calibrator::generatePreintegrator(deque<excalib::FrameData> &queue, int steps, Preintegrator2 &preint)
{
}