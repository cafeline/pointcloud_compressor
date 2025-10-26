// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_SERVICES_COMPRESSION_EXECUTOR_HPP
#define POINTCLOUD_COMPRESSOR_SERVICES_COMPRESSION_EXECUTOR_HPP

#include <string>
#include <vector>

#include "pointcloud_compressor/core/PointCloudCompressor.hpp"
#include "pointcloud_compressor/io/CompressionReportBuilder.hpp"
#include "pointcloud_compressor/io/Hdf5Writers.hpp"
#include "pointcloud_compressor/utils/ErrorAccumulator.hpp"
#include "pointcloud_compressor/bridge/Bridge.hpp"

namespace pointcloud_compressor::services {

class CompressionExecutor {
public:
    CompressionExecutor();

    PCCCompressionReport compress(const PCCCompressionRequest& request);
    void release(PCCCompressionReport& report);

private:
    pointcloud_compressor::PointCloudCompressor compressor_;
    pointcloud_compressor::io::CompressionReportBuilder report_builder_;
    pointcloud_compressor::CompressedMapData map_data_;
    std::vector<uint8_t> dictionary_buffer_;
    std::vector<uint8_t> indices_buffer_;
    std::vector<uint8_t> occupancy_buffer_;
    std::string error_message_;
    PCCCompressionReport last_report_{};

    PCCCompressionReport makeEmptyReport() const;
    PCCCompressionReport makeErrorReport(const std::string& message);
    void clearBuffers();
};

}  // namespace pointcloud_compressor::services

#endif  // POINTCLOUD_COMPRESSOR_SERVICES_COMPRESSION_EXECUTOR_HPP
