#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "Parameters.h"
#include "ExtCalibrator.h"
using namespace std;

string FILEPATH;
string PARAMNAME;

int main(int argc, char** argv){

    if(argc == 3){
        // folder name
        FILEPATH = argv[1];
        PARAMNAME = argv[2];
    }
    else{
        // online
        FILEPATH = "/home/dongshin/mywork/log_office1";
        PARAMNAME = "/home/dongshin/mywork/extrinsic_calibrator/config.xml";
    }

    Parameters PR(FILEPATH, PARAMNAME);
    
    ExtCalibrator ec(PR);
    ec.init();
    ec.run();

    return 0;
}