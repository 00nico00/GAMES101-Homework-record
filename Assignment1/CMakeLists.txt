cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

message("MSVC_VERSION: ${MSVC_VERSION}")

#set openc paths
set(OpenCV_DIR E:/opencv/opencv/build)
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)
#set eigen paths
set(EIGEN3_INCLUDE_DIR E:/Eigen3/include)

set(CMAKE_CXX_STANDARD 17)

include_directories(${OpenCV_INCLUDE_DIRS};${EIGEN3_INCLUDE_DIR})

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})