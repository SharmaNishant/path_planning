cmake_minimum_required(VERSION 3.3)
project(NCOGenerator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    NCOGenerator.cpp)

find_package( OpenCV REQUIRED )

add_executable(NCOGenerator ${SOURCE_FILES})
target_link_libraries( NCOGenerator ${OpenCV_LIBS} )






























