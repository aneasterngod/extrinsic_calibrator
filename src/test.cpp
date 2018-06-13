#include "excalib_common.h"

#include "Preintegrator.h"

int main(int argc, char** argv){

    Preintegrator preint;

    for(int i=0;i<100;i++){
        preint.addSignals(0.01, 0.0, 0.0, 0.0, 0.0, 0.45 * deg2rad, 0.1);
    }
    preint.printAll();


    return 0;
}