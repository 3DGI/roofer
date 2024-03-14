#include "projHelper.hpp"
#include "io/PointCloudReader.hpp"

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
    git_AnyUncommittedChanges() ? "dirty, " : "", 
    git_CommitDate()
  );
}


int main(int argc, const char * argv[]) {

  auto cmdl = argh::parser({ "-c", "--config" });

  cmdl.parse(argc, argv);
  std::string program_name = cmdl[0];

  std::string path_pointcloud = "/Users/ravi/git/roofer/wippolder/wippolder.las";

  // bool output_all = cmdl[{"-a", "--all"}];
  // bool write_rasters = cmdl[{"-r", "--rasters"}];
  // bool write_metadata = cmdl[{"-m", "--metadata"}];
  // bool write_index = cmdl[{"-i", "--index"}];
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

  auto pj = roofer::createProjHelper();
  auto PointReader = roofer::createPointCloudReaderLASlib(*pj);

  PointReader->open(path_pointcloud);
  spdlog::info("Reading pointcloud from {}", path_pointcloud);
  roofer::PointCollection points;
  roofer::AttributeVecMap attributes;
  PointReader->readPointCloud(points);

  spdlog::info("Read {} points", points.size());

}