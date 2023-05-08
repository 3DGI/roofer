#include "../datastructures.hpp"
#include "../pip_util.hpp"
#include "Raster.hpp"
#include "PointcloudRasteriser.hpp"

#include <numeric>

namespace roofer {

  void RasterisePointcloud(
    PointCollection& pointcloud,
    LinearRing& footprint,
    PointCloudImageBundle& image_bundle,
    // Raster& heightfield,
    float cellsize ,
    bool use_footprint
  ) {

    Box box;
    if (use_footprint) {
      box = footprint.box();
    } else {
      box = pointcloud.box();
    }
    auto boxmin = box.min();
    auto boxmax = box.max();

    RasterTools::Raster r_max(cellsize, boxmin[0], boxmax[0], boxmin[1], boxmax[1]);
    r_max.prefill_arrays(RasterTools::MAX);

    RasterTools::Raster r_min(r_max), r_fp(r_max);
    r_min.prefill_arrays(RasterTools::MIN);
    r_fp.prefill_arrays(RasterTools::MAX);

    std::vector<std::vector<float>> buckets(r_max.dimx_*r_max.dimy_);

    if (use_footprint) {
      auto exterior = build_grid(footprint);
      std::vector<pGridSet> holes;
      for (auto& hole : footprint.interior_rings()) {
        holes.push_back(build_grid(hole));
      }
      
      for (size_t col = 0; col < r_fp.dimx_; ++col) {
        for (size_t row = 0; row < r_fp.dimy_; ++row) {
          auto p = r_fp.getPointFromRasterCoords(col, row);
          pPipoint pipoint = new Pipoint{p[0],p[1]};
          if (GridTest(exterior, pipoint)) {
            r_fp.add_point(p[0], p[1], 1, RasterTools::MAX);
          } else {
            r_fp.add_point(p[0], p[1], 0, RasterTools::MAX);
          }
          for (auto& hole : holes) {
          if (GridTest(hole, pipoint)) {
            r_fp.add_point(p[0], p[1], 1, RasterTools::MAX);
          } else {
            r_fp.add_point(p[0], p[1], 0, RasterTools::MAX);
          }
          }
          delete pipoint;
        }
      }
      delete exterior;
      for (auto& hole: holes) delete hole;
    }

    for(auto& p : pointcloud) {
      if (r_max.check_point(p[0], p[1])) {
        r_max.add_point(p[0], p[1], p[2], RasterTools::MAX);
        r_min.add_point(p[0], p[1], p[2], RasterTools::MIN);
        buckets[ r_max.getLinearCoord(p[0],p[1]) ].push_back(p[2]);
      }
    }
    
    // PointCollection grid_points;
    // vec1f values;
    // double nodata = r_max.getNoDataVal();
    
    image_bundle.max.dim_x = r_max.dimx_;
    image_bundle.max.dim_y = r_max.dimy_;
    image_bundle.max.min_x = r_max.minx_;
    image_bundle.max.min_y = r_max.miny_;
    image_bundle.max.cellsize = r_max.cellSize_;
    image_bundle.max.nodataval = r_max.noDataVal_;
    image_bundle.max.array = *r_max.vals_;
    image_bundle.min = image_bundle.max;
    image_bundle.min.nodataval = r_min.noDataVal_;
    image_bundle.min.array = *r_min.vals_;
    image_bundle.fp = image_bundle.max;
    image_bundle.fp.array = *r_fp.vals_;
    image_bundle.cnt = image_bundle.max, image_bundle.med = image_bundle.max, image_bundle.avg = image_bundle.max, image_bundle.var = image_bundle.max;

    for(size_t i=0; i<r_max.dimy_ ; ++i) {
      for(size_t j=0; j<r_max.dimx_ ; ++j) {
        auto p = r_max.getPointFromRasterCoords(i,j);
        // if (p[2]!=nodata) {
          // grid_points.push_back(p);
          // values.push_back(p[2]);
        // }
        auto lc = r_max.getLinearCoord(i,j);
        auto& buck = buckets.at( lc );
        if (buck.size() == 0) {
          image_bundle.cnt.array[lc] = image_bundle.cnt.nodataval;
          image_bundle.med.array[lc] = image_bundle.med.nodataval;
          image_bundle.avg.array[lc] = image_bundle.avg.nodataval;
          image_bundle.var.array[lc] = image_bundle.var.nodataval;
        } else {
          std::sort(buck.begin(), buck.end());
          image_bundle.cnt.array[lc] = buck.size();
          image_bundle.med.array[lc] = buck[ buck.size()/2 ];
          image_bundle.avg.array[lc] = std::accumulate(buck.begin(), buck.end(), 0) / buck.size();
          int ssum = 0;
          for(auto& z : buck) {
            ssum += std::pow(z-image_bundle.avg.array[lc], 2);
          }
          image_bundle.var.array[lc] = ssum / buck.size();
        }
      }
    }
    
    // output("image_fp").set(I_fp);
    // output("image_max").set(I_max);
    // output("image_min").set(I_min);
    // output("image_cnt").set(I_cnt);
    // output("image_med").set(I_med);
    // output("image_avg").set(I_avg);
    // output("image_var").set(I_var);
    // output("heightfield").set(heightfield);
    // output("values").set(values);
    // output("grid_points").set(grid_points);
  }


}