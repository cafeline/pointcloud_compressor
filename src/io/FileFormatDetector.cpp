// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/io/FileFormatDetector.hpp"
#include "vq_occupancy_compressor/io/PcdIO.hpp"
#include "vq_occupancy_compressor/io/PlyIO.hpp"
#include <filesystem>
#include <algorithm>
#include <fstream>

namespace vq_occupancy_compressor {

FileFormat FileFormatDetector::detectFormat(const std::string& filename) {
    // Check if file exists first
    if (!std::filesystem::exists(filename)) {
        return FileFormat::UNKNOWN;
    }
    
    // First try content-based detection (more reliable)
    FileFormat content_format = detectByContent(filename);
    if (content_format != FileFormat::UNKNOWN) {
        return content_format;
    }
    
    // Fall back to extension-based detection
    return detectByExtension(filename);
}

FileFormat FileFormatDetector::detectByExtension(const std::string& filename) {
    if (filename.empty()) {
        return FileFormat::UNKNOWN;
    }
    
    std::string extension = getFileExtension(filename);
    std::string normalized = normalizeExtension(extension);
    
    if (normalized == ".pcd") {
        return FileFormat::PCD;
    }
    else if (normalized == ".ply") {
        return FileFormat::PLY;
    }
    
    return FileFormat::UNKNOWN;
}

FileFormat FileFormatDetector::detectByContent(const std::string& filename) {
    if (!std::filesystem::exists(filename)) {
        return FileFormat::UNKNOWN;
    }
    
    // Check if it's a directory
    if (std::filesystem::is_directory(filename)) {
        return FileFormat::UNKNOWN;
    }
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        return FileFormat::UNKNOWN;
    }
    
    // Read the first few lines to determine format
    std::string line;
    std::vector<std::string> first_lines;
    constexpr std::size_t lines_to_read = 5; // Read first 5 lines for analysis
    
    while (std::getline(file, line) && first_lines.size() < lines_to_read) {
        // Remove trailing whitespace
        const auto last_non_ws = line.find_last_not_of(" \n\r\t");
        if (last_non_ws == std::string::npos) {
            continue;
        }
        line.erase(last_non_ws + 1);
        first_lines.push_back(line);
    }
    
    file.close();
    
    if (first_lines.empty()) {
        return FileFormat::UNKNOWN;
    }
    
    // Check for PLY format (first line should be "ply")
    if (first_lines[0] == "ply") {
        return PlyIO::validateFormat(filename) ? FileFormat::PLY : FileFormat::UNKNOWN;
    }
    
    // Check for PCD format (usually starts with # .PCD or VERSION)
    for (const auto& line : first_lines) {
        if (line.find("# .PCD") == 0 || 
            line.find("VERSION") == 0 ||
            line.find("FIELDS") == 0 ||
            line.find("POINTS") == 0) {
            // Try to parse as PCD to validate
            PcdHeader header;
            return PcdIO::parseHeader(filename, header) ? FileFormat::PCD : FileFormat::UNKNOWN;
        }
    }
    
    return FileFormat::UNKNOWN;
}

std::string FileFormatDetector::formatToString(FileFormat format) {
    switch (format) {
        case FileFormat::PCD:
            return "PCD";
        case FileFormat::PLY:
            return "PLY";
        case FileFormat::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

bool FileFormatDetector::isSupportedFormat(FileFormat format) {
    return format == FileFormat::PCD || format == FileFormat::PLY;
}

std::string FileFormatDetector::getFileExtension(const std::string& filename) {
    if (filename.empty()) {
        return "";
    }
    
    size_t dot_pos = filename.find_last_of('.');
    size_t slash_pos = filename.find_last_of("/\\");
    
    // Check if dot is after the last slash (to handle paths like "/path/to/.hidden")
    if (dot_pos == std::string::npos || 
        (slash_pos != std::string::npos && dot_pos < slash_pos) ||
        dot_pos == filename.length() - 1) {
        return "";
    }
    
    // Handle hidden files (files starting with .) - but still consider known extensions
    size_t filename_start = (slash_pos == std::string::npos) ? 0 : slash_pos + 1;
    if (dot_pos == filename_start) {
        // This is a hidden file like ".hidden" or ".pcd"
        std::string potential_ext = filename.substr(dot_pos);
        std::string normalized_ext = normalizeExtension(potential_ext);
        
        // If it's a known file extension, return it
        if (normalized_ext == ".pcd" || normalized_ext == ".ply") {
            return potential_ext;
        }
        
        // Otherwise, check if there's another dot for real extension
        size_t second_dot = filename.find('.', dot_pos + 1);
        if (second_dot == std::string::npos) {
            return ""; // Hidden file with no known extension
        }
        return filename.substr(second_dot); // Return extension after second dot
    }
    
    return filename.substr(dot_pos);
}

std::string FileFormatDetector::normalizeExtension(const std::string& extension) {
    std::string normalized = extension;
    
    // Convert to lowercase
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), 
                   [](unsigned char c) { return std::tolower(c); });
    
    return normalized;
}

} // namespace vq_occupancy_compressor
