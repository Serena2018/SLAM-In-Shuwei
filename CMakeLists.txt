# writed by zhang ning 2018/3/22
cmake_minimum_required( VERSION 2.8 )

project(trajectory_compare)

set( CMAKE_BUILD_TYPE "Debug" )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" ) 



find_package( Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV QUIET)
#find_package(Eigen3 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}

)

set(Sophus_LIBRARIES libSophus.so)
add_executable(pangolin_with_two_threads pangolin_with_two_threads.cpp pangolin_with_two_threads.h)
target_link_libraries( pangolin_with_two_threads ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )
