cmake_minimum_required(VERSION 3.21)
project (roofer VERSION 0.1)

# ptinpoly
add_library(ptinpoly STATIC thirdparty/ptinpoly/ptinpoly.c)
set_target_properties(
  ptinpoly PROPERTIES 
  C_STANDARD 11
  POSITION_INDEPENDENT_CODE ON
)

if(EXISTS "${PROJECT_SOURCE_DIR}/.gitmodules")
  execute_process(
    COMMAND             git submodule update --init region-grower glm
    WORKING_DIRECTORY   ${PROJECT_SOURCE_DIR}/thirdparty
    )
endif()

find_package(laslib CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
add_definitions(-DCGAL_EIGEN3_ENABLED)
find_package(CGAL 5.4 QUIET COMPONENTS Core REQUIRED)

if (MSVC)
  # windows.h breaks std::min/std::max, fix by define
  add_definitions(-DNOMINMAX)
  # enable permissive compiling and/or statements
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive-")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
endif()

add_executable(roofer
  src/stepedge_nodes.cpp
  src/node_optimise_arrangement.cpp
  src/points_in_polygons.cpp
  src/point_edge.cpp
  src/region_growing.cpp
  src/node_pcmesh_quality.cpp
  src/Raster.cpp
  src/heightfield_nodes.cpp
  src/polygon_triangulate.cpp
  src/line_regulariser.cpp
  src/build_arrangement_lines.cpp
  # src/build_arrangement_rings.cpp
  src/arrangement.cpp
  src/snap_round.cpp
  src/polygon_offset.cpp
  src/alpha_shape.cpp
  src/data_coverage_node.cpp
  src/detect_planes_node.cpp
  src/polygon_util.cpp
  src/cdt_util.cpp
  src/trisnap.cpp
  src/pip_util.cpp
  src/ClusterPointCloudNode.cpp
  src/ContourRegulariserNode.cpp
  src/MaxInscribedCircleNode.cpp
  src/MeshSimplifyFastQuadNode.cpp
  src/MeshClipperNode.cpp
  src/MeshSimplify.cpp
  src/MeshGridSimplify.cpp
  src/Mesh2TriangleCollectionNode.cpp
)

target_include_directories(roofer PRIVATE
  src
  thirdparty/ptinpoly
  thirdparty/region-grower
  ${GLM_INCLUDE_DIRECTORIES}
)
target_link_libraries( roofer PRIVATE
  ptinpoly
  CGAL::CGAL CGAL::CGAL_Core Eigen3::Eigen
  LASlib
)