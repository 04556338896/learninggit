cmake_minimum_required(VERSION 3.17)
project(selectpoint)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)

add_executable(selectpoint main.cpp)
target_link_libraries(selectpoint ${freenect2_LIBRARIES})
target_link_libraries(selectpoint ${OpenCV_LIBS})
