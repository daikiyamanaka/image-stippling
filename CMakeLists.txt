project(Stippling)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
aux_source_directory(src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

set(EIGEN_DIR /Users/daikiyamanaka/Program/eigen-3.1.3)
set(CAIRO_LIB /usr/local/lib/libcairo.dylib)

######################
# Compiler flags
######################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O2 -Wall -Wextra -pedantic")

##########################
# opencv
##########################
set(OpenCV_DIR "/usr/local/opencv/share/OpenCV/OpenCVConfig.cmake"
    CACHE PATH "The path where OpenCVConfig.cmake is placed")
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
  message(STATUS "Found 'opencv library' ${OpenCV_VERSION}")
  #include_directories(${OpenCV_INCLUDE_DIRS}) #要らない？
  #link_directories(${OpenCV_LIB_DIR})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
else()
  message(SEND_ERROR "Couldn't find OpenCV")
endif()

##########
# Eigen
##########
include_directories(${EIGEN_DIR})

##########
# Cairo
##########
target_link_libraries(${PROJECT_NAME} ${CAIRO_LIB})
