cmake_minimum_required(VERSION 3.1)

find_package(CUDA REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "-O3 -Wall")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED core) # dummy
if(OpenCV_VERSION VERSION_LESS 3.0)
	find_package(OpenCV REQUIRED core highgui imgproc contrib
				PATHS /usr/local)  # 2.x
else()
	find_package(OpenCV REQUIRED core highgui imgproc imgcodecs videoio
				NO_MODULE # should be optional, tells CMake to use config mode
				PATHS /usr/local # look here
				NO_DEFAULT_PATH)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(../../include)

find_package(OpenCV REQUIRED core) # dummy
find_package(CUDA REQUIRED)

set(CUDA_ARCH "-arch=sm_50" CACHE STRING "Value of the NVCC -arch option.")
set(WITH_CUDA ON)

CUDA_ADD_EXECUTABLE(stereo_image_cv_gpumat stereosgm_image_cv_gpumat.cpp ${CUDA_SRC})

TARGET_LINK_LIBRARIES(stereo_image_cv_gpumat sgm ${CUDA_LIBRARIES} ${OpenCV_LIBS})
