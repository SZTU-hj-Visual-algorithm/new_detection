cmake_minimum_required(VERSION 3.20)
project(shsihi)

set(CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR C:/OpenCV_454/build)
set(EIGEN_INCLUDE_DIR E:/Eigen3.4.0/include/eigen3)
set(CAM_LIB ${PROJECT_SOURCE_DIR}/tools/MVCAMSDK.lib)
set(CAM_LIB64 ${PROJECT_SOURCE_DIR}/tools/MVCAMSDK_X64.lib)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/camera)


aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRC)

add_executable(shsihi main_debug.cpp)

target_link_libraries(shsihi SRCSO)
target_link_libraries(shsihi ${OpenCV_LIBS})
target_link_libraries(shsihi ${CAM_LIB})
target_link_libraries(shsihi ${CAM_LIB64})

add_library(SRCSO ${DIR_SRC})