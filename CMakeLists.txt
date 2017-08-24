


cmake_minimum_required(VERSION 3.9)

project(pointcloudmesh)
find_package(CGAL REQUIRED)
find_package(PCL 1.8 REQUIRED)

include(${CGAL_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CGAL_LIBRARY_DIRS})

add_executable (pointcloudmesh pointcloudmesh.cpp)
target_link_libraries (pointcloudmesh ${PCL_LIBRARIES}  ${CGAL_LIBS})

