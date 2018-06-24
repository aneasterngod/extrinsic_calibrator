#include <iostream>
#include <memory>
#include "Calibrator.h"
#include "GlobalParams.h"

string FILEPATH;

int main(int argc, char** argv){

    if(argc == 2){
        // folder name
        FILEPATH = argv[1];
    }
    else{
        // online
        FILEPATH = "";
    }

    
    std::shared_ptr<GlobalParams> params(new GlobalParams());

    params->setMatchingType(TRACKING);
    params->setMinimumMaintainedTrackedFeatureNumber(100);
    params->setFastThreshold(5);

    Calibrator cb;
    cb.init(params);
    cb.run();

    return 0;
}