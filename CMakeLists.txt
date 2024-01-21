cmake_minimum_required(VERSION 3.27)
project(kdtree)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


add_library(kdtree kdtree.cpp)
target_link_libraries(kdtree PUBLIC Eigen3::Eigen)


add_executable(kdtree_test main.cpp)
target_link_libraries(kdtree_test PRIVATE kdtree)
if (CMAKE_BUILD_TYPE MATCHES RELEASE)
  target_compile_options(kdtree PUBLIC -O3 -march=native)
  target_compile_options(kdtree_test PUBLIC -O3 -march=native)
endif ()


add_executable(kdtree_test_other other/main.cpp)
target_link_libraries(kdtree_test_other PUBLIC Eigen3::Eigen)
target_include_directories(kdtree_test_other PRIVATE other)
if (CMAKE_BUILD_TYPE MATCHES RELEASE)
  target_compile_options(kdtree_test_other PUBLIC -O3 -march=native)
endif ()
