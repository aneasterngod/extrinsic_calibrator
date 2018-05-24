# Dongshin Kim, 2018-05-24 23:47
include_directories (${extrinsic_calibrator_SOURCE_DIR}/src)
link_directories (${extrinsic_calibrator_BINARY_DIR}/src)
set (CORELIBS )
add_executable (excalib excalib.cpp)
target_link_libraries (excalib ${CORELIBS} )
# for debugging --- print out all variable names:
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
