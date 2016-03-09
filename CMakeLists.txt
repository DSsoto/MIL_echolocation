cmake_minimum_required(VERSION 2.8)
project(echolocation)

if (BUILD_SHARED_LIBS)
  remove_definitions(-DCERES_BUILDING_SHARED_LIBRARY)
endif()
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(echolocation hydrophone_locate_source.cpp)
target_link_libraries(echolocation ${CERES_LIBRARIES} ${OpenCV_LIBS})
