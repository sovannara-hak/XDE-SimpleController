cmake_minimum_required(VERSION 2.8)
project(SimpleController CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")

set (ROS_ROOT $ENV{ROS_ROOT})
if (ROS_ROOT)
	include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
	rosbuild_init()
	rosbuild_find_ros_package( rtt )
	set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
endif()

if(MSVC)
    set(LIB_TYPE "STATIC")
    set(suffix_type "_static")
    set(install_type "ARCHIVE")
else()
    set(LIB_TYPE "SHARED")
    set(suffix_type "")
    if(UNIX)
        set(install_type "LIBRARY")
    else()
        set(install_type "RUNTIME")
    endif()
endif()

IF (UNIX)
    ADD_DEFINITIONS(-DUNIX)
ENDIF (UNIX)

# Check dependencies
find_package(PkgConfig REQUIRED)

find_package(Orocos-RTT REQUIRED ${RTT_HINTS})

PKG_CHECK_MODULES(PC_OCL_DEPLOY ocl-deployment-gnulinux REQUIRED)
PKG_CHECK_MODULES(PC_OCL orocos-ocl-gnulinux REQUIRED)

PKG_CHECK_MODULES(PC_XDECORE xdecore REQUIRED)
if(NOT PC_XDECORE_FOUND)
    message("Failed to detect xdecore, check PKG_CONFIG_PATH." )
endif(NOT PC_XDECORE_FOUND)

PKG_CHECK_MODULES(PC_EIGEN eigen3 REQUIRED)
if(NOT PC_EIGEN_FOUND)
    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
endif(NOT PC_EIGEN_FOUND)

ADD_DEFINITIONS(-DOROCOS_TARGET=gnulinux)
# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

add_subdirectory(src)
