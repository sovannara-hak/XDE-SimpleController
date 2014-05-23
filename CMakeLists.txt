cmake_minimum_required(VERSION 2.8)
project(SimpleController CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")


# Check dependencies
find_package(PkgConfig REQUIRED)

find_package(Orocos-RTT REQUIRED ${RTT_HINTS})

PKG_CHECK_MODULES(PC_OCL_DEPLOY ocl-deployment-gnulinux REQUIRED)
PKG_CHECK_MODULES(PC_OCL orocos-ocl-gnulinux REQUIRED)

PKG_CHECK_MODULES(PC_XDECORE xdecore REQUIRED)
if(NOT PC_XDECORE_FOUND)
    message("Failed to detect xdecore, check PKG_CONFIG_PATH." )
endif(NOT PC_XDECORE_FOUND)

PKG_CHECK_MODULES(PC_EIGEN eigen_lgsm REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

ADD_DEFINITIONS(-DOROCOS_TARGET=gnulinux)
# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

add_subdirectory(src)
