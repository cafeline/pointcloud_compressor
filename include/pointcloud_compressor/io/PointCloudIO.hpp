// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_POINTCLOUD_IO_HPP
#define POINTCLOUD_COMPRESSOR_POINTCLOUD_IO_HPP

#include <string>
#include "PcdIO.hpp"
#include "PlyIO.hpp"
#include "FileFormatDetector.hpp"

namespace pointcloud_compressor {

// Unified point cloud I/O interface
class PointCloudIO {
public:
    // Load point cloud from file (auto-detect format)
    static bool loadPointCloud(const std::string& filename, PointCloud& cloud);
    
    // Save point cloud to file (format determined by extension)
    static bool savePointCloud(const std::string& filename, const PointCloud& cloud);
    
    // Load point cloud with explicit format specification
    static bool loadPointCloud(const std::string& filename, PointCloud& cloud, FileFormat format);
    
    // Save point cloud with explicit format specification
    static bool savePointCloud(const std::string& filename, const PointCloud& cloud, FileFormat format);
    
    // Get detailed information about the loaded file
    struct FileInfo {
        FileFormat detected_format;
        FileFormat extension_format;
        size_t point_count;
        bool format_mismatch;
        std::string error_message;
        
        FileInfo() : detected_format(FileFormat::UNKNOWN), 
                    extension_format(FileFormat::UNKNOWN),
                    point_count(0), 
                    format_mismatch(false) {}
    };
    
    // Load point cloud and get detailed file information
    static bool loadPointCloudWithInfo(const std::string& filename, PointCloud& cloud, FileInfo& info);
    
    // Validate file format and content without loading
    static bool validateFile(const std::string& filename);
    
    // Get supported file extensions
    static std::vector<std::string> getSupportedExtensions();
    
    // Check if filename has supported extension
    static bool hasSupportedExtension(const std::string& filename);

private:
    // Helper functions
    static bool loadPcdFile(const std::string& filename, PointCloud& cloud);
    static bool loadPlyFile(const std::string& filename, PointCloud& cloud);
    static bool savePcdFile(const std::string& filename, const PointCloud& cloud);
    static bool savePlyFile(const std::string& filename, const PointCloud& cloud);
    static std::string getErrorMessage(const std::string& operation, const std::string& filename, FileFormat format);
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_POINTCLOUD_IO_HPP