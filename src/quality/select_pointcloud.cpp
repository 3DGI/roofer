#include "select_pointcloud.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

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

  // Count the pixels that are NoData in the Image.
  size_t countNoData(const Image& img);

  float computeNoDataFraction(const CandidatePointCloud* pc);

  float computeNoDataMaxCircleFraction(const CandidatePointCloud* pc);

  // Determine if the two point clouds describe the same object.
  bool isMutated(const CandidatePointCloud* a, 
                    const CandidatePointCloud* b, 
                    const float& threshold_mutation_fraction,
                    const float& threshold_mutation_difference);

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
    std::sort(candidates_quality.begin(), candidates_quality.end(),
              roofer::compareByQuality);
    std::sort(candidates_date.begin(), candidates_date.end(),
              roofer::compareByDate);

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
      if (roofer::isMutated(best_suffcient, latest_suffcient, cfg.threshold_mutation_fraction, cfg.threshold_mutation_difference)) {
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
    float nodata = computeNoDataFraction(pc);
    bool nodata_good = nodata <= threshold_nodata;
    float nodata_maxcircle = computeNoDataMaxCircleFraction(pc);
    bool maxcircle_good = nodata_maxcircle <= threshold_maxcircle;
    return nodata_good && maxcircle_good;
  }

  bool isMutated(const CandidatePointCloud* a,
                 const CandidatePointCloud* b,
                 const float& threshold_mutation_fraction,
                 const float& threshold_mutation_difference) {
    auto footprint_mask =
        computeMask(a->image_bundle.fp.array, 0);
    auto data_mask_a =
        computeMask(a->image_bundle.max.array, a->image_bundle.max.nodataval);
    auto data_mask_b =
        computeMask(b->image_bundle.max.array, b->image_bundle.max.nodataval);

    std::vector<bool> all_mask;
    all_mask.resize(footprint_mask.size());
    std::transform(data_mask_a.begin(), data_mask_a.end(), data_mask_b.begin(),
                   all_mask.begin(), std::multiplies<>());
    std::transform(all_mask.begin(), all_mask.end(), footprint_mask.begin(),
                   all_mask.begin(), std::multiplies<>());

    std::vector<float> all_mask_on_a;
    all_mask_on_a.resize(all_mask.size());
    std::transform(a->image_bundle.max.array.begin(),
                   a->image_bundle.max.array.end(), all_mask.begin(),
                   all_mask_on_a.begin(), std::multiplies<>());
    std::vector<float> all_mask_on_b;
    all_mask_on_b.resize(all_mask.size());
    std::transform(b->image_bundle.max.array.begin(),
                   b->image_bundle.max.array.end(), all_mask.begin(),
                   all_mask_on_b.begin(), std::multiplies<>());

    // The cells are marked 'true' when there is a change between the two
    // point clouds.
    std::vector<bool> change_mask;
    change_mask.resize(all_mask_on_a.size());
    std::transform(all_mask_on_a.begin(), all_mask_on_a.end(),
                   all_mask_on_b.begin(), change_mask.begin(), 
                   [ threshold_mutation_difference ] (const float& a, const float& b) {
                    return std::abs(b - a) > threshold_mutation_difference;
                   });

    float footprint_pixel_cnt =
        std::reduce(footprint_mask.begin(), footprint_mask.end());
    float change_pixel_cnt =
        std::reduce(change_mask.begin(), change_mask.end());

    return (change_pixel_cnt / footprint_pixel_cnt) >= threshold_mutation_fraction;
  }

  std::vector<bool> computeMask(const std::vector<float>& image_array,
                                const float& nodataval) {
    std::vector<bool> mask;
    mask.reserve(image_array.size());
    for (const auto& cell : image_array) {
      if (cell == nodataval) {
        mask.push_back(false);
      } else {
        mask.push_back(true);
      }
    }
    return mask;
  }

  float computeNoDataFraction(const CandidatePointCloud* pc) {
    auto& fp = pc->image_bundle.fp.array;
    auto& cnt = pc->image_bundle.cnt.array;
    auto& cnt_nodata = pc->image_bundle.cnt.nodataval;
    size_t fp_cnt{0}, data_cnt{0};
    for (size_t i=i; fp.size(); ++i) {
      if( fp[i] ) {
        ++fp_cnt;
        if( cnt[i] != cnt_nodata ) {
          ++data_cnt;
        }
      }
    }
    if (data_cnt == 0) {
      return 0;
    } else {
      double data_frac = double(data_cnt) / double(fp_cnt);
      return float(1 - data_frac);
    }
  }

  float computeNoDataMaxCircleFraction(const CandidatePointCloud* pc) {
    if (pc->area == 0 || pc->nodata_radius == 0) {
      return 0;
    } else {
      double pi = std::atan(1) * 4;
      double nodata_area = std::pow(pc->nodata_radius, 2) * pi;
      return float(nodata_area / pc->area);
    }
  }

}  // namespace roofer