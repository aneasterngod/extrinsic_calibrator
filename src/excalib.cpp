#include <iostream>
#include <memory>
#include "Calibrator.h"
#include "GlobalParams.h"
#include "worldpt.h"

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
    params->setmineigen(0.0001);
    params->setMinfeaturedist(10);
    Eigen::Matrix3d K;
    K << 609.995173118313, 0, 382.503162687338, 0, 616.931905036158, 245.479029559645, 0, 0, 1;
    Eigen::Matrix<double, 5, 1> D;
    D << -0.135174425459394, 0.124463816968010, 0.000544090737966000, 0.00166513896881900, 0;
    params->setK(K);
    params->setD(D);

    Calibrator cb;
    cb.init(params);
    cb.run();

    return 0;
}