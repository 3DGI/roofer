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

#include "spdlog/spdlog.h"

#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

void print_help(std::string program_name) {
  // see http://docopt.org/
  std::cout << "Usage: \n";
  std::cout << "   " << program_name;
  std::cout << " [-c <file>]\n";
  // std::cout << "\n";
  std::cout << "Options:\n";
  // std::cout << "   -v, --version                Print version information\n";
  std::cout << "   -c <file>, --config <file>   Config file\n";
}

struct InputPointcloud {
  std::string path;
  std::string name;
  int quality;
  int date;
  roofer::vec1f nodata_radii;
  std::vector<roofer::PointCollection> building_clouds;
  std::vector<roofer::PointCloudImageBundle> building_rasters;
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
  if (cmdl({"-c", "--config"}) >> config_path) {
    if (!fs::exists(config_path)) {
      std::cerr << "ERROR: no such config file: " << config_path << "\n";
      print_help(program_name);
      return EXIT_FAILURE;
    }
    std::cout << "Reading configuration from file " << config_path << std::endl;
    toml::table config;
    try {
      config = toml::parse_file( config_path );
    } catch (const std::exception& e) {
      std::cerr << "ERROR: unable to parse config file " << config_path << "\n";
      std::cerr << "  " << e.what() << std::endl;
      return EXIT_FAILURE;
    }

    auto tml_path_footprint = config["input"]["footprint"]["path"].value<std::string>();
    if(tml_path_footprint.has_value())
      path_footprint = *tml_path_footprint;

    std::cout << path_footprint << std::endl;

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

        auto tml_path = (*tb)["path"].value<std::string>();
        if (tml_path.has_value()) {
          pc.path = *tml_path;
          
        }
        input_pointclouds.push_back( pc );
      };
    }
  } else {
    std::cerr << "ERROR: no config file specified\n";
    return EXIT_FAILURE;
  }

  auto pj = roofer::createProjHelper();
  auto VectorReader = roofer::createVectorReaderOGR(*pj);
  auto PointCloudCropper = roofer::createPointCloudCropper(*pj);
  auto VectorOps = roofer::createVector2DOpsGEOS();
  auto LASWriter = roofer::createLASWriter(*pj);

  VectorReader->open(path_footprint);
  auto footprints = VectorReader->readPolygons();

  // simplify + buffer footprints
  VectorOps->simplify_polygons(footprints);
  auto buffered_footprints = footprints;
  VectorOps->buffer_polygons(buffered_footprints);

  // Crop all pointclouds
  std::map<std::string, std::vector<roofer::PointCollection>> point_clouds;
  for (auto& ipc : input_pointclouds) {
    std::cout << ipc.name << std::endl;
    std::cout << ipc.path << std::endl;
    std::cout << ipc.quality << std::endl;
    PointCloudCropper->process(
      ipc.path,
      footprints,
      buffered_footprints,
      ipc.building_clouds
    );
  }

  // compute nodata maxcircle
  // compute rasters
  for (auto& ipc : input_pointclouds) {
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

  
  // TODO: write out geoflow config + pointcloud / fp for each building
  // TODO: PC selection algo
  
  for (unsigned i=0; i<footprints.size(); ++i) {
    
    std::vector<roofer::CandidatePointcloud> candidates;
    for (auto& ipc : input_pointclouds) {
      candidates.push_back(
        {
          footprints[i].signed_area(),
          ipc.nodata_radii[i],
          ipc.building_rasters[i],
          ipc.quality,
          ipc.date
        }
      );
    }

    int selection;
    roofer::PointcloudSelectExplanation explanation;
    roofer::select_pointcloud(
      candidates,
      selection,
      explanation
    );
    
    LASWriter->write_pointcloud(
      input_pointclouds[selection].building_clouds[i],
      "fmt/out/x.las"
    );

    // write config
  }




}