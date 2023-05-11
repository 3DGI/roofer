#include "projHelper.hpp"
#include "io/VectorReader.hpp"
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

#include <iostream>
#include <fstream>
#include <filesystem>
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
  std::vector<roofer::PointCloudImageBundle> building_rasters;
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
  std::string building_raster_file_spec;
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

    auto building_raster_file_spec_ = config["output"]["building_raster_file"].value<std::string>();
    if(building_raster_file_spec_.has_value())
      building_raster_file_spec = *building_raster_file_spec_;

  } else {
    spdlog::error("No config file specified\n");
    return EXIT_FAILURE;
  }

  auto pj = roofer::createProjHelper();
  auto VectorReader = roofer::createVectorReaderOGR(*pj);
  auto PointCloudCropper = roofer::createPointCloudCropper(*pj);
  auto VectorOps = roofer::createVector2DOpsGEOS();
  auto LASWriter = roofer::createLASWriter(*pj);

  VectorReader->open(path_footprint);
  spdlog::info("Reading footprints from {}", path_footprint);
  auto footprints = VectorReader->readPolygons();

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
      roofer::RasterisePointcloud(
        ipc.building_clouds[i],
        footprints[i],
        ipc.building_rasters[i]
      );
    }
  }

  
  // write out geoflow config + pointcloud / fp for each building
  spdlog::info("Selecting and writing pointclouds");
  for (unsigned i=0; i<footprints.size(); ++i) {
    
    std::vector<roofer::CandidatePointCloud> candidates;
    candidates.reserve(input_pointclouds.size());
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

    roofer::PointCloudSelectExplanation explanation;
    auto selected = roofer::selectPointCloud(candidates, explanation);
    if (!selected) {
      spdlog::info("Did not find suitable point cloud for footprint idx: {}. Skipping configuration", i);
      continue ;
    }
    // TODO: Compare PC with year of construction of footprint if available
    auto ipc_name = input_pointclouds[selected->index].name;
    auto ipc_date = input_pointclouds[selected->index].date;
    std::string pc_path = fmt::format(building_las_file_spec, fmt::arg("bid", i), fmt::arg("pc_name", ipc_name));
    std::string config_path = fmt::format(building_toml_file_spec, fmt::arg("bid", i), fmt::arg("pc_name", ipc_name));
    std::string raster_path = fmt::format(building_raster_file_spec, fmt::arg("bid", i), fmt::arg("pc_name", ipc_name));
    
    LASWriter->write_pointcloud(
      input_pointclouds[selected->index].building_clouds[i],
      pc_path
    );

    auto gf_config = toml::table {
      {"INPUT_FOOTPRINT", pc_path},
      {"INPUT_POINTCLOUD", config_path},
      {"BID", i},
      {"GROUND_ELEVATION", input_pointclouds[selected->index].ground_elevations[i]},
      {"TILE_ID", "0"},

      {"GF_PROCESS_OFFSET_OVERRIDE", true},
      {"GF_PROCESS_OFFSET_X", (*pj->data_offset)[0]},
      {"GF_PROCESS_OFFSET_Y", (*pj->data_offset)[1]},
      {"GF_PROCESS_OFFSET_Z", (*pj->data_offset)[2]},

      {"U_SOURCE", ipc_name},
      {"U_SURVEY_DATE", ipc_date},
    };
    auto tbl_gfparams = config["output"]["reconstruction_parameters"].as_table();
    gf_config.insert(tbl_gfparams->begin(), tbl_gfparams->end());

    // fs::create_directories(fs::path(fname).parent_path());

    std::ofstream ofs;
    ofs.open(config_path);
    ofs << gf_config;
    ofs.close();
    // write config
  }

}