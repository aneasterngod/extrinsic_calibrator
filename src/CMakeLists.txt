# Dongshin Kim, 2018-05-24 23:47


# opencv and eigen sophus is in the Shares directory so does not necessary to find package on them
#find_package(Ceres REQUIRED)
#find_package(PCL 1.3 REQUIRED COMPONENTS common io)
#add_definitions(${PCL_DEFINITIONS})


include_directories (
    ${extrinsic_calibrator_SOURCE_DIR}/src
    ${extrinsic_calibrator_SOURCE_DIR}/include    
    /home/dongshin/Shares/include
    /home/dongshin/Shares/include/pcl-1.8
    /usr/include/vtk-6.3
    /usr/include/eigen3
    #${PCL_INCLUDE_DIRS}
    #${extrinsic_calibrator_SOURCE_DIR}/include/custom_ceres
    #${extrinsic_calibrator_SOURCE_DIR}/include/custom_gtsam
)

link_directories (
    ${extrinsic_calibrator_BINARY_DIR}/src
    /home/dongshin/Shares/lib
    #${PCL_LIBRARY_DIRS}
)

add_executable (excalib 
    excalib.cpp
    Parameters.cpp
    ExtCalibrator.cpp
    LowPassFilter.cpp
    ImageData.cpp
    ImuData.cpp
    Frame.cpp
    Preintegrator.cpp 
    RightJacobian.cpp
    Preintegration/RelativeImuFactor.cpp
)

add_library(RelativeImuFactor SHARED
    Preintegration/RelativeImuFactor.cpp    
)

add_executable(testimu
    Preintegration/ImuFactorsExample.cpp
)

target_link_libraries(excalib
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_imgcodecs
    opencv_features2d
    opencv_calib3d 
    opencv_video
    pthread
    boost_system
    tbb
    tbbmalloc
    pcl_common
    pcl_visualization
    pcl_cuda_features
    #${CERES_LIBRARIES}  
    #${PCL_COMMON_LIBRARIES} 
    #${PCL_IO_LIBRARIES}
    #${PCL_LIBRARIES}
    gtsam      
    tinyxml2
    boost_thread  
) 

target_link_libraries(RelativeImuFactor
    gtsam
)

target_link_libraries(testimu
    pthread
    boost_system
    tbb
    tbbmalloc
    gtsam      
    boost_thread  
    RelativeImuFactor
) 

set(CMAKE_BUILD_TYPE Debug) 