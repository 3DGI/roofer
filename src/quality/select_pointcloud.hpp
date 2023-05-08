#include "../geometry/PointcloudRasteriser.hpp"

namespace roofer {

  struct CandidatePointCloud {
    float area;           // footprint area
    float nodata_radius;  // radius of the incribed circle in the largest gap
                          // in the point cloud
    int yoc;              // building year of construction
    PointCloudImageBundle image_bundle;
    std::string name;     // point cloud name
    int quality;          // point cloud quality score
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

  void selectPointCloud(std::vector<CandidatePointCloud> candidates,
                        int& selection,
                        PointCloudSelectExplanation& explanation);

  bool compareByQuality(const CandidatePointCloud& a,
                        const CandidatePointCloud& b);

  bool compareByDate(const CandidatePointCloud& a,
                     const CandidatePointCloud& b);

  // Determine if the point cloud has enough point coverage for a good
  // reconstruction.
  bool hasEnoughPointCoverage(const CandidatePointCloud& pc);

  // Determine if the two point clouds describe the same object.
  bool areDifferent(const CandidatePointCloud& a, const CandidatePointCloud& b);

}  // namespace roofer