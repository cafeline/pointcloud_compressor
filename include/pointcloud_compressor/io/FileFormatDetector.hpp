// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_FILE_FORMAT_DETECTOR_HPP
#define POINTCLOUD_COMPRESSOR_FILE_FORMAT_DETECTOR_HPP

#include <string>

namespace pointcloud_compressor {

// Supported file formats
enum class FileFormat {
    UNKNOWN,
    PCD,
    PLY
};

// File format detection utility
class FileFormatDetector {
public:
    // Detect file format by extension and content
    static FileFormat detectFormat(const std::string& filename);
    
    // Detect format by file extension only
    static FileFormat detectByExtension(const std::string& filename);
    
    // Detect format by file content (header)
    static FileFormat detectByContent(const std::string& filename);
    
    // Convert FileFormat enum to string
    static std::string formatToString(FileFormat format);
    
    // Check if file format is supported
    static bool isSupportedFormat(FileFormat format);
    
    // Get file extension from filename
    static std::string getFileExtension(const std::string& filename);

private:
    // Helper function to normalize file extensions
    static std::string normalizeExtension(const std::string& extension);
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_FILE_FORMAT_DETECTOR_HPP