// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_SERVICES_COMPRESSION_EXECUTOR_HPP
#define VQ_OCCUPANCY_COMPRESSOR_SERVICES_COMPRESSION_EXECUTOR_HPP

#include <functional>
#include <string>
#include <vector>

#include "vq_occupancy_compressor/core/VqOccupancyCompressor.hpp"
#include "vq_occupancy_compressor/report/ReportUtilities.hpp"
#include "vq_occupancy_compressor/io/Hdf5Writers.hpp"
#include "vq_occupancy_compressor/utils/ErrorAccumulator.hpp"
#include "vq_occupancy_compressor/bridge/Bridge.hpp"
#include "vq_occupancy_compressor/config/ConfigTransforms.hpp"

namespace vq_occupancy_compressor::services {

using CompressionSuccessCallback =
    std::function<void(const PCCCompressionReport&, vq_occupancy_compressor::io::CompressionReportBuilder&)>;

class CompressionExecutor {
public:
    CompressionExecutor();

    PCCCompressionReport compress(const PCCCompressionRequest& request);
    void release(PCCCompressionReport& report);

private:
    vq_occupancy_compressor::VqOccupancyCompressor compressor_;
    vq_occupancy_compressor::io::CompressionReportBuilder report_builder_;
    vq_occupancy_compressor::CompressedMapData map_data_;
    std::vector<uint8_t> dictionary_buffer_;
    std::vector<uint8_t> indices_buffer_;
    std::vector<uint8_t> occupancy_buffer_;
    std::string error_message_;
    PCCCompressionReport last_report_{};

    PCCCompressionReport makeEmptyReport() const;
    PCCCompressionReport makeErrorReport(const std::string& message);
    void clearBuffers();
};

bool runCompression(const vq_occupancy_compressor::config::CompressionSetup& setup,
                    const CompressionSuccessCallback& on_success,
                    std::string* error_message = nullptr);

}  // namespace vq_occupancy_compressor::services

#endif  // VQ_OCCUPANCY_COMPRESSOR_SERVICES_COMPRESSION_EXECUTOR_HPP
