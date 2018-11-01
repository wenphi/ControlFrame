project(controlFrameHello)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
SET(CMAKE_C_COMPILER   /usr/bin/gcc-4.8)
SET(CMAKE_CXX_COMPILER /usr/bin/g++-4.8)
add_subdirectory(src)
add_subdirectory(main)
add_subdirectory(test)


