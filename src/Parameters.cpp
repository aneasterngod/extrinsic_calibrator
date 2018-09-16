#include "Parameters.h"

Parameters::Parameters(string logfilepath, string filename){
    m_doc.LoadFile( filename.c_str() );
    
    m_stParam.pC.m_str_cameraname = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("CAMNAME")->GetText();    
    float f32Fx = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("FC")->FirstChildElement("VALUE")->FloatText();
    float f32Fy = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("FC")->FirstChildElement("VALUE")->NextSiblingElement()->FloatText();
    float f32Cu = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("CC")->FirstChildElement("VALUE")->FloatText();
    float f32Cv = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("CC")->FirstChildElement("VALUE")->NextSiblingElement()->FloatText();
    m_stParam.pC.m_mat3_K << f32Fx, 0, f32Cu, 0, f32Fy, f32Cv, 0, 0, 1;
    cv::eigen2cv(m_stParam.pC.m_mat3_K, m_stParam.pC.m_cvmat_K);
    m_stParam.pC.m_mat3_Kinv = m_stParam.pC.m_mat3_K.inverse();
    cv::eigen2cv(m_stParam.pC.m_mat3_Kinv, m_stParam.pC.m_cvmat_Kinv);
    float arf32Kc[5];
    arf32Kc[0] = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("KC")->FirstChildElement("VALUE")->FloatText();
    arf32Kc[1] = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("KC")->FirstChildElement("VALUE")->NextSiblingElement()->FloatText();
    arf32Kc[2] = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("KC")->FirstChildElement("VALUE")->NextSiblingElement()->NextSiblingElement()->FloatText();
    arf32Kc[3] = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("KC")->FirstChildElement("VALUE")->NextSiblingElement()->NextSiblingElement()->NextSiblingElement()->FloatText();
    arf32Kc[4] = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("KC")->FirstChildElement("VALUE")->NextSiblingElement()->NextSiblingElement()->NextSiblingElement()->NextSiblingElement()->FloatText();
    m_stParam.pC.m_vec5_D << arf32Kc[0], arf32Kc[1], arf32Kc[2], arf32Kc[3], arf32Kc[4];
    cv::eigen2cv(m_stParam.pC.m_vec5_D,m_stParam.pC.m_cvmat_D);
    m_stParam.pC.m_s32Width = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("IMAGEWIDTH")->IntText();
    m_stParam.pC.m_s32Height = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "CALIBRATION" )->FirstChildElement("IMAGEHEIGHT")->IntText();

    string matchingtype = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("MATCHINGTYPE")->GetText();
    if(matchingtype == "MATCHING"){
        m_stParam.pF.m_eMatchingtype = ENUM_MATCHING;
    }
    else if(matchingtype == "TRACKING"){
        m_stParam.pF.m_eMatchingtype = ENUM_TRACKING;
    }
    m_stParam.pF.m_s32MinTrackedFeatureNumbers = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("MINTRACKEDNUMBER")->IntText();
    m_stParam.pF.pFast.m_bNMS = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("FAST")->FirstChildElement("NMS")->BoolText();
    m_stParam.pF.pFast.m_f32Threshold= m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("FAST")->FirstChildElement("FASTTHRESHOLD")->FloatText();
    
    m_stParam.pF.pGFTT.m_bUseHarrisDetector = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("GFTT")->FirstChildElement("USEHARRIS")->BoolText();
    m_stParam.pF.pGFTT.m_f32K = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("GFTT")->FirstChildElement("K")->FloatText();
    m_stParam.pF.pGFTT.m_f32QualityLevel = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("GFTT")->FirstChildElement("QUALITYLEVEL")->FloatText();
    m_stParam.pF.pGFTT.m_s16Maxfeaturepoints = (int16_t)m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("GFTT")->FirstChildElement("MAXFEATURENUMBERS")->IntText();
    m_stParam.pF.pGFTT.m_s32Mindistfeature = (int32_t)m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("GFTT")->FirstChildElement("MINDISTFEATURE")->IntText();
    m_stParam.pF.pGFTT.m_s8BlockSize = (int8_t)m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("GFTT")->FirstChildElement("BLOCKSIZE")->IntText();
    m_stParam.pF.pKLT.m_f32MinEigen = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "FEATURE" )->FirstChildElement("KLT")->FirstChildElement("MINEIGEN")->FloatText();
    
    m_stParam.pL.m_f32DeltaT = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "LOWPASSFILTER" )->FirstChildElement("CUTOFFFREQ")->FloatText();
    m_stParam.pL.m_f32Freq = m_doc.FirstChildElement( "CONFIG" )->FirstChildElement( "LOWPASSFILTER" )->FirstChildElement("DELTATIME")->FloatText();
    m_stParam.m_str_logfilepath = logfilepath;

    
}

Parameters::Parameters(){

}

Parameters::~Parameters(){

}

Parameters& Parameters::operator=(const Parameters &rhs){
    m_stParam = rhs.m_stParam;
}
