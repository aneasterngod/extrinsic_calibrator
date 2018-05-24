
Description
========================================


Procedure for building code using CMake
========================================
You can either create an in-source build, or an out-of-source build. 

An out-of-source build keeps clean separation between source and build, but requires more overhead to switch between source and build directories if necesary.

An in-source build is perhaps more intuitive, but has the disadvantage of polluting your source directories with auto-generated files.

To create an out-of-source build:

	[extrinsic_calibrator]$ mkdir build

	[extrinsic_calibrator]$ cd build

	[extrinsic_calibrator/build]$ cmake ..

	[extrinsic_calibrator/build]$ make

Note that the last two commands are executed inside the build directory.

Afterwards, your tree structure should look like this:

	[extrinsic_calibrator]$ tree -L 2

	.
	|-- CMakeLists.txt
	|-- README.md
	|-- build
	|   |-- CMakeCache.txt
	|   |-- CMakeFiles
	|   |-- Makefile
	|   |-- cmake_install.cmake
	|   |-- src
	|-- src
	  |-- CMakeLists.txt
	  |-- extrinsic_calibrator

and you can execute the program in the excalib.cpp/build/src directory as follows:

	[extrinsic_calibrator/build/src]$ ./excalib

To create an in-source build,

	[extrinsic_calibrator]$ cmake .

	[extrinsic_calibrator]$ make

This will create the executables in the extrinsic_calibrator/src directory.

For more information on using CMake, check out 

	http://www.cmake.org/cmake/help/cmake_tutorial.html

Dongshin Kim, 2018-05-24 23:47
	