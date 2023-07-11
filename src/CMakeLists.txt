cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(project_with_rolling_shutter)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -std=c++14")
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
# find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Yaml-Cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(project_with_rolling_shutter
        main.cpp camera_model.cpp cloud_projector.cpp intrinsics_io.cpp
        point_cloud.cpp poses_interpolation.cpp rendering_gflags.cpp
        timestamp_loader.cpp util.cpp)
target_link_libraries(${PROJECT_NAME}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${YAML_CPP_LIBRARIES}
                      gflags glog)
