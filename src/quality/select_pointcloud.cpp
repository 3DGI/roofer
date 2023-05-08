#include "select_pointcloud.hpp"

#include <algorithm>

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
      if (roofer::hasEnoughPointCoverage(candidate)) {
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
      if (roofer::hasEnoughPointCoverage(latest)) {
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

  bool hasEnoughPointCoverage(const CandidatePointCloud& pc) {
    // TODO
    return true;
  }

  bool areDifferent(const CandidatePointCloud& a,
                    const CandidatePointCloud& b) {
    // TODO
    return true;
  }
}  // namespace roofer