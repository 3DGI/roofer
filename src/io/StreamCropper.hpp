
#include "../datastructures.hpp"
#include "../projHelper.hpp"
#include <memory>

namespace roofer {
  struct PointCloudCropperInterface {

    projHelperInterface& pjHelper;

    PointCloudCropperInterface(projHelperInterface& pjh) : pjHelper(pjh) {};

    virtual void process(
      std::string source,
      std::vector<LinearRing>& polygons,
      std::vector<LinearRing>& buf_polygons,
      std::vector<PointCollection>& point_clouds,
      vec1f& ground_elevations
    ) = 0;
  };

  std::unique_ptr<PointCloudCropperInterface> createPointCloudCropper(projHelperInterface& pjh);
}