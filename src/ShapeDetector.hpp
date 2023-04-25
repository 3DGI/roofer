#include "datastructures.hpp"
#include <memory>

namespace roofer {
// Type declarations.

struct ShapeDetectorInterface {

  virtual unsigned detectPlanes(
    PointCollection& point_collection, 
    vec3f& normals, 
    vec1i& labels,
    float probability = 0.01,
    int min_points = 15,
    float epsilon = 0.2,
    float cluster_epsilon = 0.5,
    float normal_threshold = 0.8
  ) = 0;

};

std::unique_ptr<ShapeDetectorInterface> createShapeDetector();

}