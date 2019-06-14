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
 #       ${EIGEN3_INCLUDE_DIR} ${EIGEN3_LIBS}

)
set(Sophus_LIBRARIES libSophus.so)
add_executable( trajectory_compare trajectory_compare.cpp)
target_link_libraries( trajectory_compare ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

set(Sophus_LIBRARIES libSophus.so)
add_executable( trajectory_only trajectory_only.cpp)
target_link_libraries( trajectory_only ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES})

set(Sophus_LIBRARIES libSophus.so)
add_executable( point_drawer point_drawer.cpp)
target_link_libraries( point_drawer ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES})

set(Sophus_LIBRARIES libSophus.so)
add_executable(trajectory_point_drawer trajectory_point_drawer.cpp)
target_link_libraries( trajectory_point_drawer ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

set(Sophus_LIBRARIES libSophus.so)
add_executable(pangolin_with_multithreads pangolin_with_multithreads.cpp)
target_link_libraries( pangolin_with_multithreads ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )

set(Sophus_LIBRARIES libSophus.so)
add_executable(pangolin_with_two_threads pangolin_with_two_threads.cpp pangolin_with_two_threads.h)
target_link_libraries( pangolin_with_two_threads ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES} ${OpenCV_LIBS} )
