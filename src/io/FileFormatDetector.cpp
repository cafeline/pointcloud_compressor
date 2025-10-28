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
    
    if (!std::filesystem::exists(filename)) {
        return FileFormat::UNKNOWN;
    }
    
    
    FileFormat content_format = detectByContent(filename);
    if (content_format != FileFormat::UNKNOWN) {
        return content_format;
    }
    
    
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
    
    
    if (std::filesystem::is_directory(filename)) {
        return FileFormat::UNKNOWN;
    }
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        return FileFormat::UNKNOWN;
    }
    
    
    std::string line;
    std::vector<std::string> first_lines;
    constexpr std::size_t lines_to_read = 5; 
    
    while (std::getline(file, line) && first_lines.size() < lines_to_read) {
        
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
    
    
    if (first_lines[0] == "ply") {
        return PlyIO::validateFormat(filename) ? FileFormat::PLY : FileFormat::UNKNOWN;
    }
    
    
    for (const auto& line : first_lines) {
        if (line.find("# .PCD") == 0 || 
            line.find("VERSION") == 0 ||
            line.find("FIELDS") == 0 ||
            line.find("POINTS") == 0) {
            
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
    
    
    if (dot_pos == std::string::npos || 
        (slash_pos != std::string::npos && dot_pos < slash_pos) ||
        dot_pos == filename.length() - 1) {
        return "";
    }
    
    
    size_t filename_start = (slash_pos == std::string::npos) ? 0 : slash_pos + 1;
    if (dot_pos == filename_start) {
        
        std::string potential_ext = filename.substr(dot_pos);
        std::string normalized_ext = normalizeExtension(potential_ext);
        
        
        if (normalized_ext == ".pcd" || normalized_ext == ".ply") {
            return potential_ext;
        }
        
        
        size_t second_dot = filename.find('.', dot_pos + 1);
        if (second_dot == std::string::npos) {
            return ""; 
        }
        return filename.substr(second_dot); 
    }
    
    return filename.substr(dot_pos);
}

std::string FileFormatDetector::normalizeExtension(const std::string& extension) {
    std::string normalized = extension;
    
    
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), 
                   [](unsigned char c) { return std::tolower(c); });
    
    return normalized;
}

} 
