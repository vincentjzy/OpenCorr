# cmake version requirement
cmake_minimum_required(VERSION 3.9)

# project name
project(opencoor_samples)

# configuration
set(CMAKE_CXX_STANDARD 11)




# .h files
include_directories(../src)

file(GLOB OPENCORR_CPP
     "../src/*.cpp"
)

set(SOURCES 
    ${OPENCORR_CPP}
    test_2d_dic_sift_icgn2.cpp
)

add_executable(sift_dic ${SOURCES})


# configure Eigen
set(EIGEN_DIR "/usr/include/eigen3/Eigen/")
INCLUDE_DIRECTORIES (${EIGEN_DIR})


# configure OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


# configure Open MP
find_package(OpenMP REQUIRED)




target_link_libraries( sift_dic ${OpenCV_LIBS} )
target_link_libraries(sift_dic fftw3)
target_link_libraries(sift_dic fftw3f)
target_link_libraries(sift_dic fftw3l)
target_link_libraries(sift_dic OpenMP::OpenMP_CXX)
