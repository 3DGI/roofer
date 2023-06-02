#include "../geometry/PointcloudRasteriser.hpp"

namespace roofer {

  struct CandidatePointCloud {
    float nodata_radius;  // radius of the incribed circle in the largest gap
                          // in the point cloud
    ImageMap& image_bundle;
    std::string name;     // point cloud name
    int quality;          // point cloud quality score. The lower the better the quality.
    int date;             // point cloud acquisition date
    int index;            // input point cloud index
  };

  enum PointCloudSelectExplanation {
    BAD_COVERAGE,
    BEST_SUFFICIENT,
    LATEST_SUFFICIENT
  };

  struct selectPointCloudConfig {
    // Thresholds determined from AHN3 Leiden
    // total % of no data area inside footprint 
    float threshold_nodata = 6.0;
    // max allowed nodata radius
    float threshold_maxcircle = 0.5;
    // The >=50% change was determined by analyzing the data.
    // See the Leiden, percent_diff_AHN3_2020 plot.
    float threshold_mutation_fraction = 0.5;
    // The threshold is 1.2 meters, because the accuracy of the Kadaster's
    // Dense Image Matching point cloud is about 30cm, so we are at 4 sigma.
    // float threshold_mutation_difference = 1.2;
    float threshold_mutation_difference = 1.2;
  };


  // return either 
  // 1. latest, unless there is coverage issue (case AHN 3/4, both have good quality)
  // 2. best quality, unless there is coverage issue (based on user quality rating, case Kadaster DIM/AHN)
  // In both cases poor coverage candidates are eliminated
  // Also considers mutations, ie. in case best quality candidate is mutated wrt latest latest is selected
  const CandidatePointCloud* selectPointCloud(const std::vector<CandidatePointCloud>& candidates,
                        PointCloudSelectExplanation& explanation,
                        const selectPointCloudConfig cfg = selectPointCloudConfig());

}  // namespace roofer