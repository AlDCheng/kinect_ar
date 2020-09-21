CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

PROJECT(kinect_ar)

# Build thirdparty kinect_fusion_lib first
SET("OpenCV_DIR" "/home/adcheng/Repos/opencv/build/")
ADD_SUBDIRECTORY("thirdparty/kinect_fusion_lib")


IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# CMake modules path includes freenect2 module
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Freenect2 REQUIRED)

# Eigen3 setup
FIND_PACKAGE(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
    INCLUDE_DIRECTORIES("${EIGEN3_INCLUDE_DIR}")
ENDIF (EIGEN3_INCLUDE_DIR)

# CUDA setup
FIND_PACKAGE(CUDA 8.0 REQUIRED)
IF (CUDA_FOUND)
    INCLUDE_DIRECTORIES("${CUDA_INCLUDE_DIRS}")
    # Current arch: Turing (RTX 2080)
    # SET (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++11 -gencode arch=compute_75,code=sm_75)
ENDIF ()

# Set CMake flags
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++14")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-variable")
SET (CMAKE_CXX_FLAGS_RELEASE "-O3")

# Include project files
SET (KINECT_FUSION_DIR ${PROJECT_SOURCE_DIR}/thirdparty/kinect_fusion_lib)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${KINECT_FUSION_DIR}/include
)

FILE(GLOB SRC
    "src/*.cpp"
)

FILE(GLOB HEADER
    "include/*.h"
)

ADD_EXECUTABLE(kinect_ar 
    ${SRC}
    ${HEADER}
)

target_link_libraries(kinect_ar
    ${OpenCV_LIBS} 
    ${FREENECT2_LIBRARY}
    KinectFusion
)