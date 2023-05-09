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
  // reconstruction. The point cloud has enough coverage if the indicators
  // are below the given thresholds.
  // The 'threshold_nodata' is fraction of the footprint [0.0-1.0] with nodata
  // areas (pixels). The 'threshold_maxcircle' is the fraction of the footprint
  // [0.0-1.0] covered by the maximum inscribed circle of the largest gap in the
  // point cloud.
  bool hasEnoughPointCoverage(const CandidatePointCloud& pc,
                              float threshold_nodata,
                              float threshold_maxcircle);

  // Count the pixels that are NoData in the Image.
  size_t countNoData(const Image& img);

  float computeNoDataFraction(const CandidatePointCloud& pc);

  float computeNoDataMaxCircleFraction(const CandidatePointCloud& pc);

  // Determine if the two point clouds describe the same object.
  bool areDifferent(const CandidatePointCloud& a, const CandidatePointCloud& b);

  // Compute a boolean mask that indicates that the cell has data.
  std::vector<bool> computeMask(const std::vector<float>& image_array,
                                const float& nodataval);

  // Is cell B significantly different than cell A?
  // Compares the two cell values to determine if there is a change in the
  // point cloud.
  bool isChange(float a, float b);

}  // namespace roofer