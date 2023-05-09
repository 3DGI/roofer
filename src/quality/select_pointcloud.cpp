#include "select_pointcloud.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace roofer {

  void selectPointCloud(std::vector<CandidatePointCloud> candidates,
                        int& selection,
                        PointCloudSelectExplanation& explanation) {
    CandidatePointCloud candidate_selected{};
    auto candidates_quality = candidates;
    auto candidates_date = candidates;
    std::sort(candidates_quality.begin(), candidates_quality.end(),
              roofer::compareByQuality);
    CandidatePointCloud best = candidates_quality[0];
    std::sort(candidates_date.begin(), candidates_date.end(),
              roofer::compareByDate);
    CandidatePointCloud latest = candidates_date[0];

    // Actually, the .yoc (and other footprint related values) are the same for
    // each candidate
    if (latest.yoc >= latest.date) {
      explanation = PointCloudSelectExplanation::TOO_OLD;
      // FIXME: we are returning an uninitialized 'selection'
      return;
    }
    for (const auto& candidate : candidates_quality) {
      if (roofer::hasEnoughPointCoverage(candidate, 0.0, 0.0)) {
        candidate_selected = candidate;
        break;
      }
    }
    // TODO: What's a better way to check if 'selected' has been set?
    if (candidate_selected.yoc == 0) {
      explanation = PointCloudSelectExplanation::LOW_COVERAGE;
      // FIXME: we are returning an uninitialized 'selection'
      return;
    }
    if (candidate_selected.name == latest.name) {
      explanation = PointCloudSelectExplanation::BEST_SUITABLE_QUALITY;
      // TODO: return candidate_selected
    } else {
      if (roofer::hasEnoughPointCoverage(latest, 0.0, 0.0)) {
        if (roofer::areDifferent(candidate_selected, latest)) {
          // If the two point clouds are different, that means
          // that the object has changed and the selected point
          // cloud is outdated, therefore, we have to use the
          // latest point cloud, even if it is possibly a lower
          // quality.
          explanation = PointCloudSelectExplanation::MOST_RECENT;
          candidate_selected = latest;
        } else {
          // They are not different, so keep the initially selected
          // higher-quality point cloud.
          explanation = PointCloudSelectExplanation::BEST_SUITABLE_QUALITY;
          // TODO: return candidate_selected
        }
      } else {
        explanation = PointCloudSelectExplanation::LATEST_INSUFFICIENT;
        candidate_selected = CandidatePointCloud();
      }
    }
    // TODO: What's a better way to check if 'selected' has been set?
    if (candidate_selected.yoc == 0) {
      // We got here, because the "latest" does not have enough point coverage.
      // So we check if the selected is new enough.
    } else {
      if (candidate_selected.yoc >= candidate_selected.date) {
        // Selected point cloud was too old
        explanation = PointCloudSelectExplanation::TOO_OLD;
        candidate_selected = CandidatePointCloud();
      }
    }

    for (int i = 0; i < candidates.size(); ++i) {
      if (candidates[i].name == candidate_selected.name) {
        selection = i;
      }
    }
  }

  bool compareByQuality(const CandidatePointCloud& a,
                        const CandidatePointCloud& b) {
    return a.quality < b.quality;
  }

  bool compareByDate(const CandidatePointCloud& a,
                     const CandidatePointCloud& b) {
    return a.date < b.date;
  }

  bool hasEnoughPointCoverage(const CandidatePointCloud& pc,
                              float threshold_nodata,
                              float threshold_maxcircle) {
    float nodata = computeNoDataFraction(pc);
    bool nodata_good = nodata <= threshold_nodata;
    float nodata_maxcircle = computeNoDataMaxCircleFraction(pc);
    bool maxcircle_good = nodata_maxcircle <= threshold_maxcircle;
    return nodata_good && maxcircle_good;
  }

  bool areDifferent(const CandidatePointCloud& a,
                    const CandidatePointCloud& b) {
    auto footprint_mask =
        computeMask(a.image_bundle.fp.array, a.image_bundle.fp.nodataval);
    auto data_mask_a =
        computeMask(a.image_bundle.max.array, a.image_bundle.max.nodataval);
    auto data_mask_b =
        computeMask(b.image_bundle.max.array, b.image_bundle.max.nodataval);

    std::vector<bool> all_mask;
    all_mask.resize(footprint_mask.size());
    std::transform(data_mask_a.begin(), data_mask_a.end(), data_mask_b.begin(),
                   all_mask.begin(), std::multiplies<>());
    std::transform(all_mask.begin(), all_mask.end(), footprint_mask.begin(),
                   all_mask.begin(), std::multiplies<>());

    std::vector<float> all_mask_on_a;
    all_mask_on_a.resize(all_mask.size());
    std::transform(a.image_bundle.max.array.begin(),
                   a.image_bundle.max.array.end(), all_mask.begin(),
                   all_mask_on_a.begin(), std::multiplies<>());
    std::vector<float> all_mask_on_b;
    all_mask_on_b.resize(all_mask.size());
    std::transform(b.image_bundle.max.array.begin(),
                   b.image_bundle.max.array.end(), all_mask.begin(),
                   all_mask_on_b.begin(), std::multiplies<>());

    // The cells are marked 'true' when there is a change between the two
    // point clouds.
    std::vector<bool> change_mask;
    change_mask.resize(all_mask_on_a.size());
    std::transform(all_mask_on_a.begin(), all_mask_on_a.end(),
                   all_mask_on_b.begin(), change_mask.begin(), isChange);

    float footprint_pixel_cnt =
        std::reduce(footprint_mask.begin(), footprint_mask.end());
    float change_pixel_cnt =
        std::reduce(change_mask.begin(), change_mask.end());

    // The >=50% change was determined by analyzing the data.
    // See the Leiden, percent_diff_AHN3_2020 plot.
    float threshold = 0.5;

    return (change_pixel_cnt / footprint_pixel_cnt) >= threshold;
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

  bool isChange(float a, float b) {
    // The threshold is 1.2 meters, because the accuracy of the Kadaster's
    // Dense Image Matching point cloud is about 30cm, so we are at 4 sigma.
    float threshold = 1.2;
    return std::abs(b - a) > threshold;
  }

  size_t countNoData(const Image& img) {
    size_t cnt(0);
    for (const auto& val : img.array) {
      if (val == img.nodataval) {
        cnt += 1;
      }
    }
    return cnt;
  }

  float computeNoDataFraction(const CandidatePointCloud& pc) {
    size_t nodata_cnt = roofer::countNoData(pc.image_bundle.fp);
    if (nodata_cnt == 0) {
      return 0;
    } else {
      size_t nr_cells = pc.image_bundle.fp.array.size();
      if (nr_cells == 0) {
        return 0;
      } else {
        double frac = double(nodata_cnt) / double(nr_cells);
        return float(frac);
      }
    }
  }

  float computeNoDataMaxCircleFraction(const CandidatePointCloud& pc) {
    if (pc.area == 0 || pc.nodata_radius == 0) {
      return 0;
    } else {
      double pi = std::atan(1) * 4;
      double nodata_area = std::pow(pc.nodata_radius, 2) * pi;
      return float(nodata_area / pc.area);
    }
  }

}  // namespace roofer