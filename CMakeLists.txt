cmake_minimum_required(VERSION 3.21)
project (roofer VERSION 0.1)

# ptinpoly
add_library(ptinpoly STATIC thirdparty/ptinpoly/ptinpoly.c)
set_target_properties(
  ptinpoly PROPERTIES 
  C_STANDARD 11
  POSITION_INDEPENDENT_CODE ON
)

# add polyscope
add_subdirectory(thirdparty/polyscope)

find_package(laslib CONFIG REQUIRED)
# find_package(Eigen3 CONFIG REQUIRED)
# add_definitions(-DCGAL_EIGEN3_ENABLED)
find_package(CGAL 5.4 QUIET COMPONENTS Core REQUIRED)

# GDAL
find_package( GDAL CONFIG REQUIRED )
if (WIN32) # vcpkg doesn't work when there is a version number here
  find_package(PROJ REQUIRED CONFIG)
else()
  find_package(PROJ 9.0.0 REQUIRED CONFIG)
endif()

if (MSVC)
  # windows.h breaks std::min/std::max, fix by define
  add_definitions(-DNOMINMAX)
  # enable permissive compiling and/or statements
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive-")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
endif()

add_executable(roofer
  apps/roofer.cpp
  src/common.cpp
  src/projHelper.cpp
  src/VectorReaderOGR.cpp
  src/StreamCropper.cpp
  src/Raster.cpp
  src/pip_util.cpp
  src/ShapeDetector.cpp
)
set_target_properties(
  roofer PROPERTIES 
  CXX_STANDARD 17
)

target_include_directories(roofer PRIVATE
  src
  thirdparty/ptinpoly
)
target_link_libraries( roofer PRIVATE
  polyscope
  PROJ::proj
  GDAL::GDAL
  ptinpoly
  CGAL::CGAL CGAL::CGAL_Core #Eigen3::Eigen
  LASlib
)