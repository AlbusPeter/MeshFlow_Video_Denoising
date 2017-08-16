cmake_minimum_required(VERSION 2.8)
PROJECT(VideoDenoiser)
SET(SRC_LIST main.cpp)
SET(OpenCV_DIR C:\\opencv\\build)

SET(CMAKE_BUILD_TYPE Release)		#Change Release to Debug to switch the mode.

FIND_PACKAGE(OpenCV REQUIRED)

SET(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/source)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${PROJECT_INCLUDE})

add_subdirectory(source)

ADD_EXECUTABLE(VideoDenoiser_EXE ${SRC_LIST})
TARGET_LINK_LIBRARIES(VideoDenoiser_EXE ${OpenCV_LIBS} SRC_MODULE_static)