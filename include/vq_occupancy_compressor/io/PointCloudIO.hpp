// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_POINTCLOUD_IO_HPP
#define VQ_OCCUPANCY_COMPRESSOR_POINTCLOUD_IO_HPP

#include <string>
#include "PcdIO.hpp"
#include "PlyIO.hpp"
#include "FileFormatDetector.hpp"

namespace vq_occupancy_compressor {


class PointCloudIO {
public:
    
    static bool loadPointCloud(const std::string& filename, PointCloud& cloud);
    
    
    static bool savePointCloud(const std::string& filename, const PointCloud& cloud);
    
    
    static bool loadPointCloud(const std::string& filename, PointCloud& cloud, FileFormat format);
    
    
    static bool savePointCloud(const std::string& filename, const PointCloud& cloud, FileFormat format);
    
    
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
    
    
    static bool loadPointCloudWithInfo(const std::string& filename, PointCloud& cloud, FileInfo& info);
    
    
    static bool validateFile(const std::string& filename);
    
    
    static std::vector<std::string> getSupportedExtensions();
    
    
    static bool hasSupportedExtension(const std::string& filename);

private:
    
    static bool loadPcdFile(const std::string& filename, PointCloud& cloud);
    static bool loadPlyFile(const std::string& filename, PointCloud& cloud);
    static bool savePcdFile(const std::string& filename, const PointCloud& cloud);
    static bool savePlyFile(const std::string& filename, const PointCloud& cloud);
    static std::string getErrorMessage(const std::string& operation, const std::string& filename, FileFormat format);
};

} 

#endif 