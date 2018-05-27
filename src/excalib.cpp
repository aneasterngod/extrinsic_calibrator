#include <iostream>
#include "Calibrator.h"


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

    

    Calibrator cb;
    cb.init();
    cb.run();

    return 0;
}