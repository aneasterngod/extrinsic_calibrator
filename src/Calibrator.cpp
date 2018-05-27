#include "Calibrator.h"

Calibrator::Calibrator()
{
    m_b_online = false;
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
    }
    else
    {
        m_thread_filefeeder = std::thread(&Calibrator::file_feeder,this);
        m_thread_filefeeder.join();
    }
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


void Calibrator::file_feeder(){
    cout << "File Feeder" << endl;
    while(1){
        // read first image and first imu
        excalib::ImageData imagedata = m_ar_imagedata.front();
        vector<excalib::ImuData> ar_imudata;
        while(1){
            // store imu
            excalib::ImuData imudata = m_ar_imudata.front();
            ar_imudata.push_back(imudata);
            if(imagedata.getTimestamp() < imudata.getTimestamp()){
                break;
            }
            m_ar_imudata.pop_front();
        }
        excalib::FrameData framedata;
        framedata.setImageData(imagedata);
        framedata.setArImuData(ar_imudata);
        doProcess(framedata);
    }
}

void Calibrator::online_feeder(){
    // it's more complicated, since the data is constantly obtained.
    while(1){

    }
}

void Calibrator::doProcess(excalib::FrameData& fd){

}