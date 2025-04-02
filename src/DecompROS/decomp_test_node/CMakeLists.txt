cmake_minimum_required(VERSION 2.8.3)
project(decomp_test_node)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    nav_msgs
    visualization_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(PCL REQUIRED)

include(./FindGUROBI.cmake)
include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIRS} 
    ${DECOMP_UTIL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GUROBI_INCLUDE_DIRS}
)

catkin_simple()

cs_add_executable(test_path_decomp_2d src/test_path_decomp_2d.cpp)

cs_add_executable(test_path_decomp_3d src/mpc.cpp)

target_link_libraries(test_path_decomp_3d 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${GUROBI_LIBRARIES}
)

cs_add_executable(test_seed_decomp src/test_seed_decomp.cpp)

cs_install()

cs_export()

