cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


project(playback)

add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

add_executable(main main.cpp cardetector.cpp ukf.cpp Car.cpp FrameRead.cpp)




