#include "pointcloud_compressor/cli/OptimizeWorkflow.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pointcloud_compressor/config/ConfigTransforms.hpp"
#include "pointcloud_compressor/core/BlockSizeReportFormatter.hpp"
#include "pointcloud_compressor/core/CompressionReportFormatter.hpp"
#include "pointcloud_compressor/io/CompressionReportBuilder.hpp"
#include "pointcloud_compressor/io/Hdf5Writers.hpp"
#include "pointcloud_compressor/services/CompressionExecutor.hpp"
#include "pointcloud_compressor/utils/ErrorAccumulator.hpp"

namespace pointcloud_compressor::cli {

namespace {

std::string joinErrors(const std::vector<std::string>& errors) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < errors.size(); ++i) {
    if (i != 0) {
      oss << "; ";
    }
    oss << errors[i];
  }
  return oss.str();
}

}  // namespace

OptimizeWorkflowResult runOptimizeWorkflow(const std::string& config_path) {
  config::BlockSizeOptimizationConfig opt_config =
      config::loadBlockSizeOptimizationConfigFromYaml(config_path);

  auto errors = opt_config.validate(true);
  if (!errors.empty()) {
    throw std::runtime_error(joinErrors(errors));
  }

  CompressionSettings base_settings = config::settingsFromConfig(opt_config.base);
  PointCloudCompressor compressor(base_settings);

  CompressionSettings optimal_settings =
      compressor.findOptimalSettings(opt_config.base.input_file);

  BlockSizeOptimizationResult block_result = compressor.findOptimalBlockSize(
      opt_config.base.input_file,
      opt_config.min_block_size,
      opt_config.max_block_size,
      opt_config.step_size,
      opt_config.verbose);

  if (block_result.optimal_block_size > 0) {
    optimal_settings.block_size = block_result.optimal_block_size;
  } else {
    optimal_settings.block_size = base_settings.block_size;
  }

  config::CompressorConfig final_config = opt_config.base;
  final_config.voxel_size = optimal_settings.voxel_size;
  final_config.block_size = optimal_settings.block_size;

  auto setup = config::buildCompressionSetup(final_config);
  auto setup_errors = config::validateCompressionSetup(setup);
  if (!setup_errors.empty()) {
    throw std::runtime_error(joinErrors(setup_errors));
  }

  utils::ErrorAccumulator error_acc;
  std::string compression_error;
  std::string compression_summary;

  const bool compression_success = services::runCompression(
      setup,
      [&](const PCCCompressionReport& report,
          pointcloud_compressor::io::CompressionReportBuilder& builder) {
        compression_summary = formatCompressionSummary(report);

        auto map_data = builder.toCompressedMapData(
            report, setup.settings.voxel_size, setup.settings.block_size);

        if (setup.request.save_hdf5 && setup.request.hdf5_output_path) {
          std::string err;
          if (!pointcloud_compressor::io::writeCompressedMap(
                  setup.config.hdf5_output_file, map_data, err)) {
            error_acc.add(err);
          }
        }

        if (setup.request.save_raw_hdf5 && setup.request.raw_hdf5_output_path) {
          std::string err;
          if (!pointcloud_compressor::io::writeRawVoxelGrid(
                  setup.config.raw_hdf5_output_file, report, err)) {
            error_acc.add(err);
          }
        }
      },
      &compression_error);

  if (!compression_success) {
    throw std::runtime_error("Compression failed: " + compression_error);
  }

  if (!error_acc.empty()) {
    throw std::runtime_error(error_acc.str());
  }

  OptimizeWorkflowResult result;
  result.optimal_settings = optimal_settings;
  result.block_result = block_result;
  result.verbose = opt_config.verbose;
  result.compression_summary = compression_summary;
  result.hdf5_output_file = setup.config.hdf5_output_file;
  result.raw_hdf5_output_file = setup.config.raw_hdf5_output_file;
  return result;
}

}  // namespace pointcloud_compressor::cli
