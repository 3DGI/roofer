#pragma once

#include "../datastructures.hpp"
#include <memory>
#include <unordered_map>

namespace roofer {

  void RasterisePointcloud(
    PointCollection& pointcloud,
    LinearRing& footprint,
    ImageMap& image_bundle,
    // Raster& heightfield,
    float cellsize = 0.5
  );
}