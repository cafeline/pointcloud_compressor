// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/io/PointCloudIO.hpp"
#include <filesystem>
#include <algorithm>

namespace pointcloud_compressor {

bool PointCloudIO::loadPointCloud(const std::string& filename, PointCloud& cloud) {
    cloud.clear();
    
    FileFormat format = FileFormatDetector::detectFormat(filename);
    return loadPointCloud(filename, cloud, format);
}

bool PointCloudIO::savePointCloud(const std::string& filename, const PointCloud& cloud) {
    FileFormat format = FileFormatDetector::detectByExtension(filename);
    return savePointCloud(filename, cloud, format);
}

bool PointCloudIO::loadPointCloud(const std::string& filename, PointCloud& cloud, FileFormat format) {
    cloud.clear();
    
    switch (format) {
        case FileFormat::PCD:
            return loadPcdFile(filename, cloud);
        case FileFormat::PLY:
            return loadPlyFile(filename, cloud);
        case FileFormat::UNKNOWN:
        default:
            return false;
    }
}

bool PointCloudIO::savePointCloud(const std::string& filename, const PointCloud& cloud, FileFormat format) {
    switch (format) {
        case FileFormat::PCD:
            return savePcdFile(filename, cloud);
        case FileFormat::PLY:
            return savePlyFile(filename, cloud);
        case FileFormat::UNKNOWN:
        default:
            return false;
    }
}

bool PointCloudIO::loadPointCloudWithInfo(const std::string& filename, PointCloud& cloud, FileInfo& info) {
    cloud.clear();
    
    // Initialize info
    info = FileInfo();
    
    if (!std::filesystem::exists(filename)) {
        info.error_message = "File does not exist: " + filename;
        return false;
    }
    
    // Detect formats
    info.detected_format = FileFormatDetector::detectByContent(filename);
    info.extension_format = FileFormatDetector::detectByExtension(filename);
    
    // Check for format mismatch
    if (info.detected_format != FileFormat::UNKNOWN && 
        info.extension_format != FileFormat::UNKNOWN &&
        info.detected_format != info.extension_format) {
        info.format_mismatch = true;
    }
    
    // Use detected format for loading (prefer content over extension)
    FileFormat load_format = (info.detected_format != FileFormat::UNKNOWN) ? 
                             info.detected_format : info.extension_format;
    
    bool success = loadPointCloud(filename, cloud, load_format);
    if (success) {
        info.point_count = cloud.size();
    } else {
        info.error_message = getErrorMessage("load", filename, load_format);
    }
    
    return success;
}

bool PointCloudIO::validateFile(const std::string& filename) {
    if (!std::filesystem::exists(filename)) {
        return false;
    }

    if (std::filesystem::is_directory(filename)) {
        return false;
    }

    FileFormat format = FileFormatDetector::detectFormat(filename);
    if (format == FileFormat::UNKNOWN) {
        return false;
    }

    // detectFormat() already performs lightweight header validation.
    // Avoid an additional full file load here to prevent duplicated I/O
    // (especially costly for large PLY files).
    return FileFormatDetector::isSupportedFormat(format);
}

std::vector<std::string> PointCloudIO::getSupportedExtensions() {
    return {".pcd", ".ply"};
}

bool PointCloudIO::hasSupportedExtension(const std::string& filename) {
    FileFormat format = FileFormatDetector::detectByExtension(filename);
    return FileFormatDetector::isSupportedFormat(format);
}

// Private helper functions
bool PointCloudIO::loadPcdFile(const std::string& filename, PointCloud& cloud) {
    return PcdIO::readPcdFile(filename, cloud);
}

bool PointCloudIO::loadPlyFile(const std::string& filename, PointCloud& cloud) {
    return PlyIO::readPlyFile(filename, cloud);
}

bool PointCloudIO::savePcdFile(const std::string& filename, const PointCloud& cloud) {
    return PcdIO::writePcdFile(filename, cloud);
}

bool PointCloudIO::savePlyFile(const std::string& filename, const PointCloud& cloud) {
    return PlyIO::writePlyFile(filename, cloud);
}

std::string PointCloudIO::getErrorMessage(const std::string& operation, const std::string& filename, FileFormat format) {
    std::string format_str = FileFormatDetector::formatToString(format);
    return "Failed to " + operation + " " + format_str + " file: " + filename;
}

} // namespace pointcloud_compressor
