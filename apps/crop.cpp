#include "projHelper.hpp"
#include "io/VectorReader.hpp"
#include "io/VectorWriter.hpp"
#include "io/RasterWriter.hpp"
#include "io/StreamCropper.hpp"
#include "io/LASWriter.hpp"
#include "geometry/Raster.hpp"
#include "geometry/Vector2DOps.hpp"
#include "geometry/NodataCircleComputer.hpp"
#include "geometry/PointcloudRasteriser.hpp"
#include "quality/select_pointcloud.hpp"

#include "external/argh.h"
#include "external/toml.hpp"

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
namespace fs = std::filesystem;

void print_help(std::string program_name) {
  // see http://docopt.org/
  fmt::print("Usage:\n");
  fmt::print("   {}", program_name);
  fmt::print(" [-c <file>]\n");
  // std::cout << "\n";
  fmt::print("Options:\n");
  // std::cout << "   -v, --version                Print version information\n";
  fmt::print("   -c <file>, --config <file>   Config file\n");
}

struct InputPointcloud {
  std::string path;
  std::string name;
  int quality;
  int date;
  roofer::vec1f nodata_radii;
  std::vector<roofer::PointCollection> building_clouds;
  std::vector<roofer::ImageMap> building_rasters;
  roofer::vec1f ground_elevations;
};

int main(int argc, const char * argv[]) {

  // auto cmdl = argh::parser({ "-f", "--footprint", "-p", "--pointcloud" });
  auto cmdl = argh::parser({ "-c", "--config" });

  cmdl.parse(argc, argv);
  std::string program_name = cmdl[0];

  // std::string vector_file = "/home/ravi/git/gfc-building-reconstruction/test-data/wippolder.gpkg";
  // std::string las_source = "/home/ravi/git/gfc-building-reconstruction/test-data/wippolder.las";
  std::string path_footprint; // = "/mnt/Data/LocalData/Kadaster/true_ortho_experimenten/2021_LAZ_Leiden_Almere/DenHaag/bag_83000_455000.gpkg";
  // std::string path_pointcloud = "/mnt/Data/LocalData/Kadaster/true_ortho_experimenten/2021_LAZ_Leiden_Almere/DenHaag/83000_455000.laz";
  std::vector<InputPointcloud> input_pointclouds;

  // TOML config parsing
  // pointclouds, footprints
  std::string config_path;
  std::string building_toml_file_spec;
  std::string building_las_file_spec;
  std::string building_gpkg_file_spec;
  std::string building_raster_file_spec;
  std::string output_path;
  std::string building_bid_attribute;
  toml::table config;
  if (cmdl({"-c", "--config"}) >> config_path) {
    if (!fs::exists(config_path)) {
      spdlog::error("No such config file: {}", config_path);
      print_help(program_name);
      return EXIT_FAILURE;
    }
    spdlog::info("Reading configuration from file {}", config_path);
    try {
      config = toml::parse_file( config_path );
    } catch (const std::exception& e) {
      spdlog::error("Unable to parse config file {}.\n{}", config_path, e.what());
      return EXIT_FAILURE;
    }

    auto tml_path_footprint = config["input"]["footprint"]["path"].value<std::string>();
    if(tml_path_footprint.has_value())
      path_footprint = *tml_path_footprint;

    auto tml_bid = config["input"]["footprint"]["bid"].value<std::string>();
    if(tml_bid.has_value())
      building_bid_attribute = *tml_bid;

    auto tml_pointclouds = config["input"]["pointclouds"];
    if (toml::array* arr = tml_pointclouds.as_array())
    {
      // visitation with for_each() helps deal with heterogeneous data
      for(auto& el : *arr)
      {
        toml::table * tb = el.as_table();
        InputPointcloud pc;
        
        pc.name = *(*tb)["name"].value<std::string>();
        pc.quality = *(*tb)["quality"].value<int>();
        pc.date = *(*tb)["date"].value<int>();

        auto tml_path = (*tb)["path"].value<std::string>();
        if (tml_path.has_value()) {
          pc.path = *tml_path;
          
        }
        input_pointclouds.push_back( pc );
      };
    }

    auto building_toml_file_spec_ = config["output"]["building_toml_file"].value<std::string>();
    if(building_toml_file_spec_.has_value())
      building_toml_file_spec = *building_toml_file_spec_;

    auto building_las_file_spec_ = config["output"]["building_las_file"].value<std::string>();
    if(building_las_file_spec_.has_value())
      building_las_file_spec = *building_las_file_spec_;
    
    auto building_gpkg_file_spec_ = config["output"]["building_gpkg_file"].value<std::string>();
    if(building_gpkg_file_spec_.has_value())
      building_gpkg_file_spec = *building_gpkg_file_spec_;

    auto building_raster_file_spec_ = config["output"]["building_raster_file"].value<std::string>();
    if(building_raster_file_spec_.has_value())
      building_raster_file_spec = *building_raster_file_spec_;
      
    auto output_path_ = config["output"]["path"].value<std::string>();
    if(output_path_.has_value())
      output_path = *output_path_;

  } else {
    spdlog::error("No config file specified\n");
    return EXIT_FAILURE;
  }

  auto pj = roofer::createProjHelper();
  auto VectorReader = roofer::createVectorReaderOGR(*pj);
  auto VectorWriter = roofer::createVectorWriterOGR(*pj);
  auto RasterWriter = roofer::createRasterWriterGDAL(*pj);
  auto PointCloudCropper = roofer::createPointCloudCropper(*pj);
  auto VectorOps = roofer::createVector2DOpsGEOS();
  auto LASWriter = roofer::createLASWriter(*pj);

  VectorReader->open(path_footprint);
  spdlog::info("Reading footprints from {}", path_footprint);
  std::vector<roofer::LinearRing> footprints;
  roofer::AttributeVecMap attributes;
  VectorReader->readPolygons(footprints, &attributes);

  // simplify + buffer footprints
  spdlog::info("Simplifying and buffering footprints...");
  VectorOps->simplify_polygons(footprints);
  auto buffered_footprints = footprints;
  VectorOps->buffer_polygons(buffered_footprints);

  // Crop all pointclouds
  std::map<std::string, std::vector<roofer::PointCollection>> point_clouds;
  for (auto& ipc : input_pointclouds) {
    spdlog::info("Cropping pointcloud {}...", ipc.name);
    PointCloudCropper->process(
      ipc.path,
      footprints,
      buffered_footprints,
      ipc.building_clouds,
      ipc.ground_elevations
    );
  }

  // compute nodata maxcircle
  // compute rasters
  for (auto& ipc : input_pointclouds) {
    spdlog::info("Analysing pointcloud {}...", ipc.name);
    unsigned N = ipc.building_clouds.size();
    ipc.nodata_radii.resize(N);
    ipc.building_rasters.resize(N);
    for(unsigned i=0; i<N; ++i) {
      roofer::compute_nodata_circle(
        ipc.building_clouds[i],
        footprints[i],
        &ipc.nodata_radii[i]
      );      
      // ipc.nodata_radii[i]=0;
      roofer::RasterisePointcloud(
        ipc.building_clouds[i],
        footprints[i],
        ipc.building_rasters[i]
      );
    }
  }

  
  // write out geoflow config + pointcloud / fp for each building
  spdlog::info("Selecting and writing pointclouds");
  auto bid_vec = attributes.get_if<std::string>(building_bid_attribute);
  auto& pc_select = attributes.insert_vec<std::string>("pc_select");
  std::string bid;
  for (unsigned i=0; i<footprints.size(); ++i) {

    if (bid_vec) {
      bid = (*bid_vec)[i];
    } else {
      bid = std::to_string(i);
    }
    
    std::vector<roofer::CandidatePointCloud> candidates;
    candidates.reserve(input_pointclouds.size());
    {
      int j=0;
      for (auto& ipc : input_pointclouds) {
        candidates.push_back(
          roofer::CandidatePointCloud {
            footprints[i].signed_area(),
            ipc.nodata_radii[i],
            // 0, // TODO: get footprint year of construction
            ipc.building_rasters[i],
            ipc.name,
            ipc.quality,
            ipc.date,
            j++
          }
        );
      }
    }
    const roofer::CandidatePointCloud* selected = nullptr;
    if(input_pointclouds.size()>1) {
      roofer::PointCloudSelectExplanation explanation;
      selected = roofer::selectPointCloud(candidates, explanation);
      if (!selected) {
        spdlog::info("Did not find suitable point cloud for footprint idx: {}. Skipping configuration", bid);
        continue ;
      }
      if (explanation == roofer::PointCloudSelectExplanation::BEST_SUFFICIENT )
        pc_select.push_back("BEST_SUFFICIENT");
      else if (explanation == roofer::PointCloudSelectExplanation::LATEST_SUFFICIENT )
        pc_select.push_back("LATEST_SUFFICIENT");
      else if (explanation == roofer::PointCloudSelectExplanation::BAD_COVERAGE )
        pc_select.push_back("BAD_COVERAGE");
    } else {
      pc_select.push_back("NA");
    }
    // TODO: Compare PC with year of construction of footprint if available
    if (selected) spdlog::info("Selecting pointcloud: {}", input_pointclouds[selected->index].name);
    
    bool only_write_selected = false;
    {
      // fs::create_directories(fs::path(fname).parent_path());
      std::string fp_path = fmt::format(building_gpkg_file_spec, fmt::arg("bid", bid), fmt::arg("path", output_path));
      VectorWriter->writePolygons(fp_path, footprints, attributes, i, i+1);

      size_t j=0;
      auto selected_index = selected ? selected->index : -1;
      for (auto& ipc : input_pointclouds) { 
        if(selected_index != j && only_write_selected) continue;

        std::string pc_path = fmt::format(building_las_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        std::string raster_path = fmt::format(building_raster_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        
        RasterWriter->writeBands(
          raster_path,
          ipc.building_rasters[i]
        );

        LASWriter->write_pointcloud(
          input_pointclouds[j].building_clouds[i],
          pc_path
        );

        auto gf_config = toml::table {
          {"INPUT_FOOTPRINT", fp_path},
          {"INPUT_POINTCLOUD", pc_path},
          {"BID", bid},
          {"GROUND_ELEVATION", input_pointclouds[j].ground_elevations[i]},
          {"TILE_ID", "0"},

          {"GF_PROCESS_OFFSET_OVERRIDE", true},
          {"GF_PROCESS_OFFSET_X", (*pj->data_offset)[0]},
          {"GF_PROCESS_OFFSET_Y", (*pj->data_offset)[1]},
          {"GF_PROCESS_OFFSET_Z", (*pj->data_offset)[2]},

          {"U_SOURCE", ipc.name},
          {"U_SURVEY_DATE", ipc.date},
        };
        auto tbl_gfparams = config["output"]["reconstruction_parameters"].as_table();
        gf_config.insert(tbl_gfparams->begin(), tbl_gfparams->end());

        std::ofstream ofs;
        std::string config_path = fmt::format(building_toml_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        ofs.open(config_path);
        ofs << gf_config;
        ofs.close();
        if(selected_index == j) {
          std::ofstream ofs;
          std::string config_path = fmt::format(building_toml_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", "optimal"), fmt::arg("path", output_path));
          ofs.open(config_path);
          ofs << gf_config;
          ofs.close();
        }
        ++j;
      }
    }
    // write config
  }
  std::string fp_path = output_path + "/index.gpkg";
  VectorWriter->writePolygons(fp_path, footprints, attributes);

}