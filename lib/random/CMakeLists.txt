cmake_minimum_required(VERSION 2.8)
project(random)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#
# Find packages.
#

find_package(Eigen3 REQUIRED)

#
# Set include directories.
#

include_directories(include ${EIGEN3_INCLUDE_DIR})


#
# Add libraries.
#

add_library(rng src/rng.cpp)

add_library(multivariate_gaussian src/multivariate_gaussian.cpp)
target_link_libraries(multivariate_gaussian rng)

#
# Add executables.
#

add_executable(test_rng test/test_rng.cpp)
target_link_libraries(test_rng rng)

add_executable(test_multivariate_gaussian test/test_multivariate_gausian.cpp)
target_link_libraries(test_multivariate_gaussian multivariate_gaussian)
