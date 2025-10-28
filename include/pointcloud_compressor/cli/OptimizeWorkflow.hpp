#ifndef POINTCLOUD_COMPRESSOR_CLI_OPTIMIZE_WORKFLOW_HPP
#define POINTCLOUD_COMPRESSOR_CLI_OPTIMIZE_WORKFLOW_HPP

#include <string>

#include "pointcloud_compressor/config/CompressorConfig.hpp"
#include "pointcloud_compressor/core/PointCloudCompressor.hpp"

namespace pointcloud_compressor::cli {

struct OptimizeWorkflowResult {
  CompressionSettings optimal_settings;
  BlockSizeOptimizationResult block_result;
  bool verbose = false;
  std::string compression_summary;
  std::string hdf5_output_file;
  std::string raw_hdf5_output_file;
};

OptimizeWorkflowResult runOptimizeWorkflow(const std::string& config_path);

}  // namespace pointcloud_compressor::cli

#endif  // POINTCLOUD_COMPRESSOR_CLI_OPTIMIZE_WORKFLOW_HPP
