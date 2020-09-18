CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

PROJECT(kinect_ar)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# CMake modules path includes freenect2 module
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Freenect2 REQUIRED)

# Set CMake flags
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++14")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-variable")
SET (CMAKE_CXX_FLAGS_RELEASE "-O3")

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
)

FILE(GLOB SRC
    "src/*.cpp"
)

FILE(GLOB HEADER
    "include/*.h"
)

ADD_EXECUTABLE(kinect_ar ${SRC} ${HEADER})

target_link_libraries(kinect_ar
  ${OpenCV_LIBS} 
  ${FREENECT2_LIBRARY}
)