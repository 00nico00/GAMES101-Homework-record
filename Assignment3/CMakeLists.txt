cmake_minimum_required(VERSION 3.10)
project(Rasterizer3)

set(CMAKE_CXX_STANDARD 17)

#find_package(OpenCV REQUIRED)
include_directories(./../)
include_directories(E:/opencv/opencv/build/include)
set(OpenCV_LIBRARIES E:/opencv/opencv/build/x64/vc15/lib/opencv_world460d.lib)
file(COPY E:/opencv/opencv/build/x64/vc15/lib/opencv_world460d.lib DESTINATION ${PROJECT_BINARY_DIR}/Debug/)

add_executable(Rasterizer3 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer3 ${OpenCV_LIBRARIES})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)

file(COPY ${PROJECT_SOURCE_DIR}/models/ DESTINATION ${PROJECT_BINARY_DIR}/../models/)