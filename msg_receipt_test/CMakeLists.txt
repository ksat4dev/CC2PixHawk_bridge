cmake_minimum_required(VERSION 3.10.2)

##set project name 
project(zmq)

##specify C++ standard and other flags globally
set(CMAKE_CXX_FLAGS "-Wall -lboost_iostreams -lboost_system -lboost_filesystem -pthread -std=c++14")

#load in pkg-config support
find_package(PkgConfig)
## use pkgconfig to get hint for zmq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from above to find where 'zmq.hpp' is located 
find_path(ZeroMQ_INCLUDE_DIR
            NAMES zmq.hpp
            PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
         )

## use the information from above to find the location of libzmq
find_library(ZeroMQ_LIBRARY
            NAMES zmq
            PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
)           

# include directory containing zhelpers.hpp
include_directories(include)                 # No target, simply the directory containing header files

# create a target that you want to include_dir and link library
add_executable(zmq_pub src/publisher.cpp)    # target_Name and source_file location

add_executable(zmq_sub src/subscriber.cpp)

#include dir containing zmq.hpp
target_include_directories(zmq_pub PUBLIC ${ZeroMQ_INCLUDE_DIR})
target_include_directories(zmq_sub PUBLIC ${ZeroMQ_INCLUDE_DIR})

# link zmq library location to the created target
target_link_libraries(zmq_pub ${ZeroMQ_LIBRARY})
target_link_libraries(zmq_sub ${ZeroMQ_LIBRARY})