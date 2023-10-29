cmake_minimum_required(VERSION 2.8.9)
PROJECT(ad936x_iiostream_switch)
# select build type to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")
########################################################################
# Find libiio
########################################################################
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(libiio 0.9 REQUIRED)
find_package(libad9361-iio 0.2 REQUIRED)
if (LIBIIO_FOUND)
    message(STATUS "LIBIIO_INCLUDE_DIRS: ${LIBIIO_INCLUDE_DIRS}")
    message(STATUS "LIBIIO_LIBRARIES: ${LIBIIO_LIBRARIES}")
    message(STATUS "LIBIIO_DEFINITIONS: ${LIBIIO_DEFINITIONS}")
endif()

if (LIBAD9361_FOUND)
    message(STATUS "LIBAD9361_INCLUDE_DIRS: ${LIBAD9361_INCLUDE_DIRS}")
    message(STATUS "LIBAD9361_LIBRARIES: ${LIBAD9361_LIBRARIES}")
    message(STATUS "LIBAD9361_DEFINITIONS: ${LIBAD9361_DEFINITIONS}")
endif()

include_directories(${FFTW3_INCLUDE_DIRS})
#enable c++11 features
if(CMAKE_COMPILER_IS_GNUCXX)

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #Thread support enabled (not the same as -lpthread)
    list(APPEND LIBIIO_LIBRARIES -pthread)
    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)
endif(CMAKE_COMPILER_IS_GNUCXX)

AUX_SOURCE_DIRECTORY(. DIR_SRCS)
ADD_EXECUTABLE(ad936x_iiostream_switch ${DIR_SRCS})
TARGET_LINK_LIBRARIES(ad936x_iiostream_switch ${LIBIIO_LIBRARIES})


