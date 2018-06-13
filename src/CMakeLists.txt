# Dongshin Kim, 2018-05-24 23:47

include_directories (
    ${extrinsic_calibrator_SOURCE_DIR}/src
    ${extrinsic_calibrator_SOURCE_DIR}/include
    /home/dongshin/Shares/include
    /usr/include/eigen3
)

link_directories (
    ${extrinsic_calibrator_BINARY_DIR}/src
    /home/dongshin/Shares/lib
)

add_executable (excalib 
    excalib.cpp
    Calibrator.cpp    
    ImageData.cpp
    ImuData.cpp
    FrameData.cpp
    Preintegrator2.cpp 
    RightJacobian.cpp
    LowPassFilter.cpp
)

add_executable(testexcalib
    test.cpp
    Preintegrator.cpp    
)

target_link_libraries(excalib
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_imgcodecs
    pthread
)

target_link_libraries(testexcalib
    opencv_core
    opencv_highgui
    opencv_imgproc
    pthread
)