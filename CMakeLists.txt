cmake_minimum_required(VERSION 3.10)
project(drake_kuka)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package (Eigen3 3.3 REQUIRED)
# INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )

# find_package(Threads MODULE REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)

add_executable(main src/main.cpp)
# target_include_directories(incline_plane)
target_link_libraries(main drake::drake Eigen3::Eigen)