# Simple
project(BspUgh)

set(
    SOURCE_LIST
    Q3Loader.h
    Q3Loader.cpp
    Q3Loader_Test.cpp
    Trace.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_executable(BspUgh ${SOURCE_LIST})


