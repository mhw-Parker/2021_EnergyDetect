cmake_minimum_required(VERSION 2.8)

project(MASTER)
SET(CMAKE_BUILE_TYPE DEBUG)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -g -std=c++14 -pthread -lv4l1 -lv4l2")
SET(BIN_NAME "run")

FIND_PACKAGE(OpenCV 4.4.0 REQUIRED)
FIND_PACKAGE(Threads)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Thread/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Serials/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Armor/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Other/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Pose/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Filter/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Energy/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Drivers/V4L2KAS/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Drivers/RealSense/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Drivers/DAHUA/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Drivers/VideoDriver/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/Drivers)

INCLUDE_DIRECTORIES(/usr/include/eigen3)

MESSAGE(STATUS "Project:MASTER")
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")
MESSAGE(STATUS "OpenCV Library Status:")
MESSAGE(STATUS "OpenCV Version: ${OpenCV_VERSION}")
MESSAGE(STATUS "OpenCV Libraries: ${OpenCVLIBS}")
MESSAGE(STATUS "Include Path:${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}")

LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/Drivers/DAHUA/lib)

FILE(GLOB_RECURSE sourcefiles "Other/src/*.cpp" "Energy/src/*.cpp" "Armor/src/*.cpp" "Serials/src/*.cpp" "Thread/src/*.cpp"
        "Pose/src/*.cpp" "Filter/src/*.cpp" "Drivers/VideoDriver/src/*.cpp")

#when "Drivers/DAHUA/src/*.cpp" is added to the sourcefiles at last line, these .cpp files looks like not been complied
ADD_EXECUTABLE(${BIN_NAME} main.cpp ${sourcefiles} "Drivers/DAHUA/src/RMDriver.cpp" "Drivers/DAHUA/src/RGBConvert.cpp" "Drivers/DAHUA/src/StreamRetrieve.cpp")

LIST(APPEND PSMOVEAPI_TRACKER_REQUIRED_LIBS v4l2)

TARGET_LINK_LIBRARIES(${BIN_NAME} ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(${BIN_NAME} ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(${BIN_NAME} MVSDK RecordVideo avcodec avfilter avformat avutil postproc swresample swscale x264 ImageConvert rt pthread)
