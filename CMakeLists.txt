cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)

project(CameraCalibration)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

set(CORE_SOURCES src/CameraCalibration.cpp src/settings.cpp src/utils.cpp)

add_executable(CameraCalibration src/main.cpp ${CORE_SOURCES})
add_executable(DistortCorrection src/DistortCorrection.cpp ${CORE_SOURCES})

target_include_directories(CameraCalibration PUBLIC include/)
target_include_directories(DistortCorrection PUBLIC include/)

set(BOOST_LIB_LIST -lboost_system -lboost_filesystem)

target_link_libraries(CameraCalibration ${OpenCV_LIBS} ${Boost_LIBRARIES} ${BOOST_LIB_LIST})
target_link_libraries(DistortCorrection ${OpenCV_LIBS} ${Boost_LIBRARIES} ${BOOST_LIB_LIST})
