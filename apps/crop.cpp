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

#include "git.h"

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
namespace fs = std::filesystem;

void print_help(std::string program_name) {
  // see http://docopt.org/
  fmt::print("Usage:\n");
  fmt::print("   {}", program_name);
  fmt::print(" -c <file>\n");
  fmt::print("Options:\n");
  // std::cout << "   -v, --version                Print version information\n";
  fmt::print("   -h, --help                   Show this help message\n");
  fmt::print("   -V, --version                Show version\n");
  fmt::print("   -v, --verbose                Be more verbose\n");
  fmt::print("   -c <file>, --config <file>   Config file\n");
  fmt::print("   -r, --rasters                Output rasterised building pointclouds\n");
  fmt::print("   -m, --metadata               Output metadata.json file\n");
  fmt::print("   -i, --index                  Output index.gpkg file\n");
  fmt::print("   -a, --all                    Output files for each candidate point cloud instead of only the optimal candidate\n");
}

void print_version() {
  fmt::print("roofer {} ({}{}{})\n", 
  git_Describe(), 
  git_Branch() == "main" ? "" : fmt::format("{}, ", git_Branch()), 
  git_AnyUncommittedChanges() ? "dirty, " : "", git_CommitDate());
}

struct InputPointcloud {
  std::string path;
  std::string name;
  int quality;
  int date;
  int bld_class = 6;
  int grnd_class = 2;
  roofer::vec1f nodata_radii;
  roofer::vec1f nodata_fractions;
  roofer::vec1f pt_densities;
  roofer::vec1b is_mutated;
  std::vector<roofer::LinearRing> nodata_circles;
  std::vector<roofer::PointCollection> building_clouds;
  std::vector<roofer::ImageMap> building_rasters;
  roofer::vec1f ground_elevations;
};

int main(int argc, const char * argv[]) {

  // auto cmdl = argh::parser({ "-f", "--footprint", "-p", "--pointcloud" });
  auto cmdl = argh::parser({ "-c", "--config" });

  cmdl.parse(argc, argv);
  std::string program_name = cmdl[0];

  std::string path_footprint; // = "/mnt/Data/LocalData/Kadaster/true_ortho_experimenten/2021_LAZ_Leiden_Almere/DenHaag/bag_83000_455000.gpkg";

  bool output_all = cmdl[{"-a", "--all"}];
  bool write_rasters = cmdl[{"-r", "--rasters"}];
  bool write_metadata = cmdl[{"-m", "--metadata"}];
  bool write_index = cmdl[{"-i", "--index"}];
  bool verbose = cmdl[{"-v", "--verbose"}];
  bool version = cmdl[{"-V", "--version"}];

  if (cmdl[{"-h", "--help"}]) {
    print_help(program_name);
    return EXIT_SUCCESS;
  }
  if (version) {
    print_version();
    return EXIT_SUCCESS;
  }

  if (verbose) {
    spdlog::set_level(spdlog::level::debug);
  } else {
    spdlog::set_level(spdlog::level::warn);
  }

  std::vector<InputPointcloud> input_pointclouds;

  // TOML config parsing
  // pointclouds, footprints
  std::string config_path;
  std::string building_toml_file_spec;
  std::string building_las_file_spec;
  std::string building_gpkg_file_spec;
  std::string building_raster_file_spec;
  std::string building_jsonl_file_spec;
  std::string jsonl_list_file_spec;
  std::string index_file_spec;
  std::string metadata_json_file_spec;
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
        
        if( auto n = (*tb)["name"].value<std::string>(); n.has_value() ){
            pc.name = *n;
        }
        if( auto n = (*tb)["quality"].value<int>(); n.has_value() ){
            pc.quality = *n;
        }
        if( auto n = (*tb)["date"].value<int>(); n.has_value() ){
            pc.date = *n;
        }

        if( auto n = (*tb)["building_class"].value<int>(); n.has_value() ){
            pc.bld_class = *n;
        }
        if( auto n = (*tb)["ground_class"].value<int>(); n.has_value() ){
            pc.grnd_class = *n;
        }

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

    if (write_metadata) {
      auto metadata_json_file_spec_ = config["output"]["metadata_json_file"].value<std::string>();
      if(metadata_json_file_spec_.has_value())
        metadata_json_file_spec = *metadata_json_file_spec_;
    }

    auto building_jsonl_file_spec_ = config["output"]["building_jsonl_file"].value<std::string>();
    if(building_jsonl_file_spec_.has_value())
      building_jsonl_file_spec = *building_jsonl_file_spec_;

    auto index_file_spec_ = config["output"]["index_file"].value<std::string>();
    if(index_file_spec_.has_value())
      index_file_spec = *index_file_spec_;
      
    auto jsonl_list_file_spec_ = config["output"]["jsonl_list_file"].value<std::string>();
    if(jsonl_list_file_spec_.has_value())
      jsonl_list_file_spec = *jsonl_list_file_spec_;

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
      ipc.ground_elevations,
      {
        .ground_class = ipc.grnd_class, 
        .building_class = ipc.bld_class
      }
    );
  }

  // compute nodata maxcircle
  // compute rasters
  const unsigned N_fp = footprints.size();
  for (auto& ipc : input_pointclouds) {
    spdlog::info("Analysing pointcloud {}...", ipc.name);
    ipc.nodata_radii.resize(N_fp);
    ipc.building_rasters.resize(N_fp);
    ipc.nodata_fractions.resize(N_fp);
    ipc.pt_densities.resize(N_fp);
    if (write_index) ipc.nodata_circles.resize(N_fp);

    // auto& r_nodata = attributes.insert_vec<float>("r_nodata_"+ipc.name);
    roofer::arr2f nodata_c;
    for(unsigned i=0; i<N_fp; ++i) {
      roofer::compute_nodata_circle(
        ipc.building_clouds[i],
        footprints[i],
        &ipc.nodata_radii[i],
        &nodata_c
      );
      if (write_index) {
        roofer::draw_circle(
          ipc.nodata_circles[i],
          ipc.nodata_radii[i],
          nodata_c
        );
      }
      // r_nodata.push_back(ipc.nodata_radii[i]);
      
      roofer::RasterisePointcloud(
        ipc.building_clouds[i],
        footprints[i],
        ipc.building_rasters[i]
      );

      ipc.nodata_fractions[i] = roofer::computeNoDataFraction(ipc.building_rasters[i]);
      ipc.pt_densities[i] = roofer::computePointDensity(ipc.building_rasters[i]);
    }
  }

  // add raster stats attributes to footprints
  for (auto& ipc : input_pointclouds) {
    auto& nodata_r = attributes.insert_vec<float>("nodata_r_"+ipc.name);
    auto& nodata_frac = attributes.insert_vec<float>("nodata_frac_"+ipc.name);
    auto& pt_density = attributes.insert_vec<float>("pt_density_"+ipc.name);
    nodata_r.reserve(N_fp);
    nodata_frac.reserve(N_fp);
    pt_density.reserve(N_fp);
    for (unsigned i=0; i<N_fp; ++i) {
      nodata_r.push_back(ipc.nodata_radii[i]);
      nodata_frac.push_back(ipc.nodata_fractions[i]);
      pt_density.push_back(ipc.pt_densities[i]);
    }
  }

  roofer::selectPointCloudConfig select_pc_cfg;
  if (input_pointclouds.size() > 1){
    auto& is_mutated = attributes.insert_vec<bool>(
      "is_mutated_"+input_pointclouds[0].name+"_"+input_pointclouds[1].name
    );
    is_mutated.reserve(N_fp);
    for (unsigned i=0; i<N_fp; ++i) {
      is_mutated[i] = roofer::isMutated(
        input_pointclouds[0].building_rasters[i], 
        input_pointclouds[1].building_rasters[i], 
        select_pc_cfg.threshold_mutation_fraction, 
        select_pc_cfg.threshold_mutation_difference
      );
    }
  }
  
  // write out geoflow config + pointcloud / fp for each building
  spdlog::info("Selecting and writing pointclouds");
  auto bid_vec = attributes.get_if<std::string>(building_bid_attribute);
  auto& pc_select = attributes.insert_vec<std::string>("pc_select");
  auto& pc_source = attributes.insert_vec<std::string>("pc_source");
  std::unordered_map<std::string, roofer::vec1s> jsonl_paths;
  std::string bid;
  bool only_write_selected = !output_all;
  for (unsigned i=0; i<N_fp; ++i) {

    if (bid_vec) {
      bid = (*bid_vec)[i].value();
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
        jsonl_paths.insert({ipc.name, roofer::vec1s{}});
      }
      jsonl_paths.insert({"", roofer::vec1s{}});
    }
    const roofer::CandidatePointCloud* selected = nullptr;
    if(input_pointclouds.size()>1) {
      roofer::PointCloudSelectExplanation explanation;
      selected = roofer::selectPointCloud(candidates, explanation, select_pc_cfg);
      if (explanation == roofer::PointCloudSelectExplanation::BEST_SUFFICIENT )
        pc_select.push_back("BEST_SUFFICIENT");
      else if (explanation == roofer::PointCloudSelectExplanation::LATEST_SUFFICIENT )
        pc_select.push_back("LATEST_SUFFICIENT");
      else if (explanation == roofer::PointCloudSelectExplanation::BAD_COVERAGE )
        pc_select.push_back("BAD_COVERAGE");
      if (!selected) {
        // spdlog::info("Did not find suitable point cloud for footprint idx: {}. Skipping configuration", bid);
        continue ;
      }
    } else {
      pc_select.push_back("NA");
      selected = &candidates[0];
    }
    pc_source.push_back(selected->name);
    // TODO: Compare PC with year of construction of footprint if available
    // if (selected) spdlog::info("Selecting pointcloud: {}", input_pointclouds[selected->index].name);
    
    {
      // fs::create_directories(fs::path(fname).parent_path());
      std::string fp_path = fmt::format(building_gpkg_file_spec, fmt::arg("bid", bid), fmt::arg("path", output_path));
      VectorWriter->writePolygons(fp_path, footprints, attributes, i, i+1);

      size_t j=0;
      auto selected_index = selected ? selected->index : -1;
      for (auto& ipc : input_pointclouds) {
        if((selected_index != j) && (only_write_selected)) {
          ++j;
          continue;
        };

        std::string pc_path = fmt::format(building_las_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        std::string raster_path = fmt::format(building_raster_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        std::string jsonl_path = fmt::format(building_jsonl_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
        
        if (write_rasters) {
          RasterWriter->writeBands(
            raster_path,
            ipc.building_rasters[i]
          );
        }

        LASWriter->write_pointcloud(
          input_pointclouds[j].building_clouds[i],
          pc_path
        );
        
        // Correct ground height for offset, NB this ignores crs transformation
        double h_ground = input_pointclouds[j].ground_elevations[i] + (*pj->data_offset)[2];
        
        auto gf_config = toml::table {
          {"INPUT_FOOTPRINT", fp_path},
          {"INPUT_POINTCLOUD", pc_path},
          {"BID", bid},
          {"GROUND_ELEVATION", h_ground},
          {"OUTPUT_JSONL", jsonl_path },

          {"GF_PROCESS_OFFSET_OVERRIDE", true},
          {"GF_PROCESS_OFFSET_X", (*pj->data_offset)[0]},
          {"GF_PROCESS_OFFSET_Y", (*pj->data_offset)[1]},
          {"GF_PROCESS_OFFSET_Z", (*pj->data_offset)[2]},

          {"U_SOURCE", ipc.name},
          {"U_SURVEY_DATE", std::to_string(ipc.date)},
        };

        if (write_metadata) {
            // gf_config.insert("GF_PROCESS_OFFSET_OVERRIDE", true);
            gf_config.insert("CITYJSON_TRANSLATE_X", (*pj->data_offset)[0]);
            gf_config.insert("CITYJSON_TRANSLATE_Y", (*pj->data_offset)[1]);
            gf_config.insert("CITYJSON_TRANSLATE_Z", (*pj->data_offset)[2]);
            gf_config.insert("CITYJSON_SCALE_X", 0.001);
            gf_config.insert("CITYJSON_SCALE_Y", 0.001);
            gf_config.insert("CITYJSON_SCALE_Z", 0.001);
        }
        auto tbl_gfparams = config["output"]["reconstruction_parameters"].as_table();
        gf_config.insert(tbl_gfparams->begin(), tbl_gfparams->end());

        if(!only_write_selected) {
          std::ofstream ofs;
          std::string config_path = fmt::format(building_toml_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ipc.name), fmt::arg("path", output_path));
          ofs.open(config_path);
          ofs << gf_config;
          ofs.close();

          jsonl_paths[ipc.name].push_back( jsonl_path );
        }
        if(selected_index == j) {
          // set optimal jsonl path
          std::string jsonl_path = fmt::format(building_jsonl_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ""), fmt::arg("path", output_path));
          gf_config.insert_or_assign("OUTPUT_JSONL", jsonl_path);
          jsonl_paths[""].push_back( jsonl_path );

          // write optimal config
          std::ofstream ofs;
          std::string config_path = fmt::format(building_toml_file_spec, fmt::arg("bid", bid), fmt::arg("pc_name", ""), fmt::arg("path", output_path));
          ofs.open(config_path);
          ofs << gf_config;
          ofs.close();
        }
        ++j;
      }
    }
    // write config
  }

  // Write index output
  if (write_index) {
    std::string index_file = fmt::format(index_file_spec, fmt::arg("path", output_path));
    VectorWriter->writePolygons(index_file, footprints, attributes);
    for (auto& ipc : input_pointclouds) {
      VectorWriter->writePolygons(index_file+"_"+ipc.name+"_nodatacircle.gpkg", ipc.nodata_circles, attributes);
    }
  }

  // write the txt containing paths to all jsonl features to be written by reconstruct
  {
    
    for(auto& [name, pathsvec] : jsonl_paths) {
      if(pathsvec.size()!=0) {
        std::string jsonl_list_file = fmt::format(jsonl_list_file_spec, fmt::arg("path", output_path), fmt::arg("pc_name", name));
        std::ofstream ofs;
        ofs.open(jsonl_list_file);
        for(auto& jsonl_p : pathsvec) {
          ofs << jsonl_p << "\n";
        }
        ofs.close();
      }
    }
  }

  // Write metadata.json for json features 
  if (write_metadata) {  
    auto md_scale = roofer::arr3d{0.001, 0.001, 0.001};
    auto md_trans = *pj->data_offset;

    auto metadatajson = toml::table{
      { "type", "CityJSON" },
      { "version", "1.1" },
      { "CityObjects", toml::table{} },
      { "vertices", toml::array{} },
      { "transform", toml::table{
          { "scale", toml::array{md_scale[0], md_scale[1], md_scale[2]} },
          { "translate", toml::array{md_trans[0], md_trans[1], md_trans[2]} },
        }
      },
      { "metadata", toml::table{
          { "referenceSystem", "https://www.opengis.net/def/crs/EPSG/0/7415" }
        }
      }
    };
    // serializing as JSON using toml::json_formatter:
    std::string metadata_json_file = fmt::format(metadata_json_file_spec, fmt::arg("path", output_path));
    
    // minimize json
    std::stringstream ss;
    ss << toml::json_formatter{ metadatajson };
    auto s = ss.str();
    s.erase(std::remove(s.begin(), s.end(), '\n'), s.cend());
    s.erase(std::remove(s.begin(), s.end(), ' '), s.cend());

    std::ofstream ofs;
    ofs.open(metadata_json_file);
    ofs << s;
    ofs.close();
  }
  return EXIT_SUCCESS;;
}