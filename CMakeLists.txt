#cmake_minimum_required(VERSION 3.5)
#
#project(demo)
#
#SET(BIN_NAME "run")
##SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
##set(THREADS_PREFER_PTHREAD_FLAG ON)
#
#
#find_package(OpenCV REQUIRED)
##find_package(Threads REQUIRED)
#find_package(X11 REQUIRED)
#
#
#
#set(CMAKE_CXX_STANDARD 14)
#
##find_package(Threads)
#
#
#message(STATUS "OpenCV library status:")
#message(STATUS "    version: ${OpenCV_VERSION}")
##message(STATUS "    libraries: ${OpenCV_DIR}")
#message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#
#message(STATUS "Eigen3 library status:")
#message(STATUS "    version: ${Eigen3_VERSION}")
##message(STATUS "    libraries: ${Eigen3_DIR}")
#message(STATUS "    include path: ${EIGEN3_INCLUDE_DIRS}")
#
#message(STATUS "    include path11: ${Threads_INCLUDE_DIRS}")
#
#include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${EIGEN3_INCLUDE_DIRS})
#include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/include/camera)
##include_directories(${Threads_INCLUDE_DIRS})
#
#file(GLOB_RECURSE sourcefiles "src/*.cpp")
#
##add_compile_options(-Wall -lX11)
#
#ADD_EXECUTABLE(${BIN_NAME} main.cpp ${sourcefiles} ${X11_INCLUDE_DIR})
#add_definitions(-std=c++11)
#
#
#set_property(TARGET ${BIN_NAME} PROPERTY CXX_STANDARD 14)
#
#target_link_libraries(${BIN_NAME} ${X11_LIBRARIES})
#
##target_link_libraries(${BIN_NAME} Threads::Threads)
#
#target_link_libraries(${BIN_NAME} ${OpenCV_LIBS})
##TARGET_LINK_LIBRARIES(${BIN_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${BIN_NAME} /lib/libMVSDK.so)
##target_link_libraries(${BIN_NAME} ${EIGEN3_LIBS})
##target_link_libraries( snack Common -lpthread -lm)
#
##ADD_CUSTOM_command(TARGET ${BIN_NAME} COMMAND "${PROJECT_SOURCE_DIR}/tools/create-startup.sh" "${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/tools")
#
cmake_minimum_required(VERSION 3.20)
project(shsihi)

set(CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR /home/lmx/Opencv454/opencv-4.5.4/build)

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/camera)

aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRC)

add_executable(shsihi main.cpp)
target_link_libraries(shsihi ${OpenCV_LIBS})



target_link_libraries(shsihi SRCSO)
add_library(SRCSO ${DIR_SRC})


target_link_libraries(shsihi /lib/libMVSDK.so)
