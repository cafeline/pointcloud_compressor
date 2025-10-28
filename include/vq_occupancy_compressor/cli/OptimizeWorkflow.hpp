#ifndef VQ_OCCUPANCY_COMPRESSOR_CLI_OPTIMIZE_WORKFLOW_HPP
#define VQ_OCCUPANCY_COMPRESSOR_CLI_OPTIMIZE_WORKFLOW_HPP

#include <string>

#include "vq_occupancy_compressor/config/CompressorConfig.hpp"
#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"

namespace vq_occupancy_compressor::cli {

struct OptimizeWorkflowResult {
  CompressionSettings optimal_settings;
  BlockSizeOptimizationResult block_result;
  bool verbose = false;
  std::string compression_summary;
  std::string hdf5_output_file;
  std::string raw_hdf5_output_file;
};

OptimizeWorkflowResult runOptimizeWorkflow(const std::string& config_path);

}  // namespace vq_occupancy_compressor::cli

#endif  // VQ_OCCUPANCY_COMPRESSOR_CLI_OPTIMIZE_WORKFLOW_HPP
