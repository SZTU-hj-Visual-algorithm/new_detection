cmake_minimum_required(VERSION 3.20)
project(shsihi)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRC)

add_executable(shsihi main.cpp)

target_link_libraries(shsihi ${OpenCV_LIBS})

target_link_libraries(shsihi SRCSO)

add_library(SRCSO ${DIR_SRC})