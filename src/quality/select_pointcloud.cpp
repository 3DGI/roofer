#include "select_pointcloud.hpp"

#include <cmath>
#include <numeric>
#include <algorithm>

// #include "spdlog/spdlog.h"

namespace roofer {

  // out the one with lowest quality val on top
  bool compareByQuality(const CandidatePointCloud* a,
                        const CandidatePointCloud* b) {
    return a->quality < b->quality;
  }

  // out the one with highest (most recent) date on top
  bool compareByDate(const CandidatePointCloud* a,
                     const CandidatePointCloud* b) {
    return a->date > b->date;
  }

  // Determine if the point cloud has enough point coverage for a good
  // reconstruction. The point cloud has enough coverage if the indicators
  // are below the given thresholds.
  // The 'threshold_nodata' is fraction of the footprint [0.0-1.0] with nodata
  // areas (pixels). The 'threshold_maxcircle' is the fraction of the footprint
  // [0.0-1.0] covered by the maximum inscribed circle of the largest gap in the
  // point cloud.
  bool hasEnoughPointCoverage(const CandidatePointCloud* pc,
                              float threshold_nodata,
                              float threshold_maxcircle);

  // Compute a boolean mask that indicates that the cell has data.
  std::vector<bool> computeMask(const std::vector<float>& image_array,
                                const float& nodataval);

  const CandidatePointCloud* selectPointCloud(const std::vector<CandidatePointCloud>& candidates,
                                        PointCloudSelectExplanation& explanation,
                                        const selectPointCloudConfig cfg) {
    std::vector<const CandidatePointCloud*> candidates_quality;
    std::vector<const CandidatePointCloud*> candidates_date;
    for(auto& cand : candidates) {
      candidates_quality.push_back(&cand);
      candidates_date.push_back(&cand);
    }
    CandidatePointCloud* candidate_selected;
    std::sort(candidates_date.begin(), candidates_date.end(),
              roofer::compareByDate);
    if(candidates_date[0]->building_yoc != -1)
      if(candidates_date[0]->building_yoc > candidates_date[0]->date) {
        explanation = PointCloudSelectExplanation::PC_OUTDATED;
        return nullptr;
      }

    std::sort(candidates_quality.begin(), candidates_quality.end(),
              roofer::compareByQuality);
    // Actually, the .yoc (and other footprint related values) are the same for
    // each candidate
    // if (latest.yoc >= latest.date) {
    //   explanation = PointCloudSelectExplanation::TOO_OLD;
    //   return;
    // }
    // get the highest quality candidate with sufficient coverage
    const CandidatePointCloud* best_suffcient = nullptr;
    // int candidates_quality_idx(-1);
    for (unsigned i = 0; i < candidates_quality.size(); ++i) {
      if (roofer::hasEnoughPointCoverage(
              candidates_quality[i], cfg.threshold_nodata, cfg.threshold_maxcircle)) {
        best_suffcient = candidates_quality[i];
        // candidates_quality_idx = i;
        break;
      }
    }
    // no candidate has sufficient coverage
    if (!best_suffcient) {
      explanation = PointCloudSelectExplanation::BAD_COVERAGE;
      return nullptr;
    }
    // get the latest candidate with sufficient coverage
    const CandidatePointCloud* latest_suffcient = nullptr;
    // int candidates_latest_idx(-1);
    for (unsigned i = 0; i < candidates_date.size(); ++i) {
      if (roofer::hasEnoughPointCoverage(
              candidates_date[i], cfg.threshold_nodata, cfg.threshold_maxcircle)) {
        latest_suffcient = candidates_date[i];
        // candidates_latest_idx = i;
        break;
      }
    }

    // highest quality is also latest pointcloud
    if (best_suffcient == latest_suffcient) {
      explanation = PointCloudSelectExplanation::BEST_SUFFICIENT;
      return best_suffcient;
    // check for mutations
    } else {
      // check if latest PC has enough coverage
      // if (roofer::hasEnoughPointCoverage(latest_suffcient, cfg.threshold_nodata,
                                        //  cfg.threshold_maxcircle)) {
      if (roofer::isMutated(best_suffcient->image_bundle, latest_suffcient->image_bundle, cfg.threshold_mutation_fraction, cfg.threshold_mutation_difference)) {
        // If the two point clouds are different, that means
        // that the object has changed and the selected point
        // cloud is outdated, therefore, we have to use the
        // latest point cloud, even if it is possibly a lower
        // quality.
        explanation = PointCloudSelectExplanation::LATEST_SUFFICIENT;
        return latest_suffcient;
      } else {
        // return the best point cloud, it seems to be not mutated in the latest_sufficient 
        explanation = PointCloudSelectExplanation::BEST_SUFFICIENT;
        return best_suffcient;
      }
        // They are not different, so keep the initially selected
        // higher-quality point cloud.
      // } else {
      //   // In this case, the latest point cloud is not even suitable for mutation detection,
      //   // because it is missing most of its points. So we need to rely on the
      //   // year of construction of the object to check if we can use the point
      //   // cloud.
      //   if (candidate_selected.yoc >= candidate_selected.date) {
      //     // Selected point cloud was too old
      //     explanation = PointCloudSelectExplanation::LATEST_INSUFFICIENT;
      //     candidate_selected = CandidatePointCloud();
      //     for (unsigned i = candidates_quality_idx;
      //          i < candidates_quality.size(); ++i) {
      //       auto candidate = candidates_quality[i];
      //       if (candidate.yoc < candidate.date) {
      //         explanation = PointCloudSelectExplanation::BEST_SUITABLE_QUALITY;
      //         candidate_selected = candidate;
      //         break;
      //       }
      //     }
      //   }
      // }
    }
  }

  bool hasEnoughPointCoverage(const CandidatePointCloud* pc,
                              float threshold_nodata,
                              float threshold_maxcircle) {
    float nodata = roofer::computeNoDataFraction(pc->image_bundle);
    bool nodata_good = nodata <= threshold_nodata;
    // float nodata_maxcircle = computeNoDataMaxCircleFraction(pc);
    bool maxcircle_good = pc->nodata_radius <= threshold_maxcircle;
    return nodata_good && maxcircle_good;
  }

}  // namespace roofer