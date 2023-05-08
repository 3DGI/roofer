
#include "../datastructures.hpp"
#include <memory>

namespace roofer {
  struct PointCloudImageBundle {
    Image fp;
    Image max;
    Image min;
    Image cnt;
    Image med;
    Image avg;
    Image var;
  };

  void RasterisePointcloud(
    PointCollection& pointcloud,
    LinearRing& footprint,
    PointCloudImageBundle& image_bundle,
    // Raster& heightfield,
    float cellsize = 0.5,
    bool use_footprint = false
  );
}