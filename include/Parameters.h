#ifndef ParameterReader_H_
#define ParameterReader_H_

#include <iostream>
#include <tinyxml2.h>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

typedef enum
{
    ENUM_MATCHING = 1,
    ENUM_TRACKING = 2
} MatchingType;

typedef struct{
    float m_f32Threshold;
    bool m_bNMS;
}paramFAST;

typedef struct{
    int16_t m_s16Maxfeaturepoints;
    float m_f32QualityLevel;
    int32_t m_s32Mindistfeature;
    int8_t m_s8BlockSize;
    bool m_bUseHarrisDetector;
    float m_f32K;
}paramGFTT;

typedef struct{
    float m_f32MinEigen;
}paramKLT;


typedef struct{    
    MatchingType m_eMatchingtype;    
    int32_t m_s32MinTrackedFeatureNumbers;
    paramFAST pFast;
    paramGFTT pGFTT;
    paramKLT pKLT;
}paramFeature;
    
    
typedef struct{
    string m_str_cameraname;
    Eigen::Matrix3d m_mat3_K;
    Eigen::Matrix3d m_mat3_Kinv;
    Eigen::Matrix<double, 5, 1> m_vec5_D;
    cv::Mat m_cvmat_K;
    cv::Mat m_cvmat_Kinv;
    cv::Mat m_cvmat_D;
    int32_t m_s32Width;
    int32_t m_s32Height;
}paramCameraCalibration;

typedef struct{
    float m_f32Freq;
    float m_f32DeltaT;
}paramLowpass;

typedef struct{
    paramFeature pF;
    paramCameraCalibration pC;
    paramLowpass pL;
    string m_str_logfilepath;
}paramProgram;

class Parameters{
    public:
        Parameters(string logfilepath, string filename);
        Parameters();
        ~Parameters();        
        Parameters& operator=(const Parameters &rhs);
    public:
        paramProgram m_stParam;
    private:
        tinyxml2::XMLDocument m_doc;
};


#endif