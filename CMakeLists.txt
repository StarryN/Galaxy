cmake_minimum_required(VERSION 2.8.3)
project(2dconvex)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

file(GLOB ${PROJECT_NAME}_SRCS src/main.cpp)

add_executable(demo
    ${${PROJECT_NAME}_SRCS})

target_link_libraries(demo ${OpenCV_LIBS})
