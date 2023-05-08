#include "../geometry/PointcloudRasteriser.hpp"

namespace roofer {
  
  struct CandidatePointcloud {
    float area;
    float nodata_radius;
    PointCloudImageBundle image_bundle;
    int quality;
    int date;
  };

  enum PointcloudSelectExplanation {
    BEST_SUITABLE_QUALITY,
    MOST_RECENT
  };

  void select_pointcloud(
    std::vector<CandidatePointcloud> candidates,
    int& selection,
    PointcloudSelectExplanation& explanation
  );

}