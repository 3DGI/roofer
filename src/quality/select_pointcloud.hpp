#include "../geometry/PointcloudRasteriser.hpp"

namespace roofer {

  struct CandidatePointCloud {
    float area;           // footprint area
    float nodata_radius;  // radius of the incribed circle in the largest gap
                          // in the point cloud
    int yoc;              // building year of construction
    PointCloudImageBundle image_bundle;
    std::string name;     // point cloud name
    int quality;          // point cloud quality score. The lower the better the quality.
    int date;             // point cloud acquisition date
  };

  enum PointCloudSelectExplanation {
    // highest quality, latest available
    BEST_SUITABLE_QUALITY,
    // highest available quality is outdated, latest selected
    MOST_RECENT,
    // object was constructed in the same year or after the acquisition of the
    // latest point cloud
    TOO_OLD,
    // none of the available point clouds have enough point coverage
    LOW_COVERAGE,
    // highest available quality is not the latest, latest has insufficient
    // coverage
    LATEST_INSUFFICIENT,
  };

  struct selectPointCloudConfig {
    // Thresholds determined from AHN3 Leiden
    float threshold_nodata = 6.0;
    float threshold_maxcircle = 4.7;
    // The >=50% change was determined by analyzing the data.
    // See the Leiden, percent_diff_AHN3_2020 plot.
    float threshold_mutation_fraction = 0.5;
    // The threshold is 1.2 meters, because the accuracy of the Kadaster's
    // Dense Image Matching point cloud is about 30cm, so we are at 4 sigma.
    // float threshold_mutation_difference = 1.2;
    float threshold_mutation_difference = 1.2;
  };

  void selectPointCloud(std::vector<CandidatePointCloud> candidates,
                        int& selection,
                        PointCloudSelectExplanation& explanation,
                        const selectPointCloudConfig cfg = selectPointCloudConfig());

}  // namespace roofer