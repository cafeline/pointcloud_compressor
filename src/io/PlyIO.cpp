#include "pointcloud_compressor/io/PlyIO.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>

namespace pointcloud_compressor {

bool PlyIO::readPlyFile(const std::string& filename, PointCloud& cloud) {
    cloud.clear();
    
    // Open file in binary mode for binary PLY files
    PlyHeader temp_header;
    std::ifstream header_file(filename);
    if (!parseHeaderInternal(header_file, temp_header)) {
        std::cerr << "Failed to parse PLY header: " << filename << std::endl;
        return false;
    }
    header_file.close();
    
    // Reopen file with appropriate mode
    std::ifstream file;
    if (isBinaryFormat(temp_header.format)) {
        file.open(filename, std::ios::binary);
    } else {
        file.open(filename);
    }
    
    if (!file.is_open()) {
        std::cerr << "Failed to open PLY file: " << filename << std::endl;
        return false;
    }
    
    PlyHeader header;
    if (!parseHeaderInternal(file, header)) {
        std::cerr << "Failed to parse PLY header: " << filename << std::endl;
        return false;
    }
    
    // Check for unsupported formats
    if (header.format == "binary_big_endian") {
        std::cerr << "Binary big endian PLY format is not supported: " << filename << std::endl;
        return false;
    }
    
    if (header.format != "ascii" && !isBinaryFormat(header.format)) {
        std::cerr << "Unsupported PLY format: " << header.format << " in file: " << filename << std::endl;
        return false;
    }
    
    // Validate that we have x, y, z properties
    if (!isValidPropertySet(header.properties)) {
        std::cerr << "PLY file must have x, y, z properties: " << filename << std::endl;
        return false;
    }
    
    if (header.format == "ascii") {
        return readAsciiData(file, cloud, header);
    } else {
        return readBinaryData(file, cloud, header);
    }
}

bool PlyIO::writePlyFile(const std::string& filename, const PointCloud& cloud) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open PLY file for writing: " << filename << std::endl;
        return false;
    }
    
    if (!writeHeader(file, cloud)) {
        std::cerr << "Failed to write PLY header: " << filename << std::endl;
        return false;
    }
    
    return writeAsciiData(file, cloud);
}

bool PlyIO::parseHeader(const std::string& filename, PlyHeader& header) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    return parseHeaderInternal(file, header);
}

bool PlyIO::validateFormat(const std::string& filename) {
    if (!std::filesystem::exists(filename)) {
        return false;
    }
    
    // First check header format
    PlyHeader temp_header;
    std::ifstream header_file(filename);
    if (!parseHeaderInternal(header_file, temp_header)) {
        return false;
    }
    header_file.close();
    
    // Reopen with appropriate mode
    std::ifstream file;
    if (isBinaryFormat(temp_header.format)) {
        file.open(filename, std::ios::binary);
    } else {
        file.open(filename);
    }
    
    if (!file.is_open()) {
        return false;
    }
    
    // Try to parse the header completely to validate format
    PlyHeader header;
    if (!parseHeaderInternal(file, header)) {
        return false;
    }
    
    // Skip validation for unsupported formats
    if (header.format == "binary_big_endian") {
        return false;
    }
    
    // Additional validation: check if we can read the expected number of vertices
    if (header.vertex_count > 0) {
        PointCloud temp_cloud;
        if (header.format == "ascii") {
            return readAsciiData(file, temp_cloud, header);
        } else if (isBinaryFormat(header.format)) {
            return readBinaryData(file, temp_cloud, header);
        }
        return false;
    }
    
    return true; // Valid PLY with 0 vertices
}

bool PlyIO::parseHeaderInternal(std::ifstream& file, PlyHeader& header) {
    std::string line;
    
    // Read and validate first line
    if (!std::getline(file, line)) {
        return false;
    }
    line.erase(line.find_last_not_of(" \n\r\t") + 1);
    if (line != "ply") {
        return false;
    }
    
    bool found_format = false;
    bool found_vertex_element = false;
    bool found_end_header = false;
    
    while (std::getline(file, line)) {
        // Remove trailing whitespace
        line.erase(line.find_last_not_of(" \n\r\t") + 1);
        
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        
        if (keyword == "format") {
            std::string format_type, version;
            iss >> format_type >> version;
            header.format = format_type;
            header.version = version;
            found_format = true;
        }
        else if (keyword == "comment") {
            std::string comment;
            std::getline(iss, comment);
            if (!comment.empty() && comment[0] == ' ') {
                comment = comment.substr(1); // Remove leading space
            }
            header.comments.push_back(comment);
        }
        else if (keyword == "element") {
            std::string element_type;
            int count;
            iss >> element_type >> count;
            if (element_type == "vertex") {
                header.vertex_count = count;
                found_vertex_element = true;
            }
        }
        else if (keyword == "property") {
            std::string property_line;
            std::getline(iss, property_line);
            header.properties.push_back(keyword + property_line);
        }
        else if (keyword == "end_header") {
            found_end_header = true;
            break;
        }
    }
    
    return found_format && found_vertex_element && found_end_header;
}

bool PlyIO::readAsciiData(std::ifstream& file, PointCloud& cloud, const PlyHeader& header) {
    cloud.points.reserve(header.vertex_count);
    
    std::string line;
    int points_read = 0;
    
    while (std::getline(file, line) && points_read < header.vertex_count) {
        line.erase(line.find_last_not_of(" \n\r\t") + 1);
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        float x, y, z;
        
        if (!(iss >> x >> y >> z)) {
            std::cerr << "Failed to parse vertex data at line " << points_read + 1 << std::endl;
            return false;
        }
        
        cloud.points.emplace_back(x, y, z);
        points_read++;
    }
    
    if (points_read != header.vertex_count) {
        std::cerr << "Expected " << header.vertex_count << " vertices, but read " << points_read << std::endl;
        return false;
    }
    
    return true;
}

bool PlyIO::writeHeader(std::ofstream& file, const PointCloud& cloud) {
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "comment Created by pointcloud_compressor\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";
    
    return file.good();
}

bool PlyIO::writeAsciiData(std::ofstream& file, const PointCloud& cloud) {
    for (const auto& point : cloud.points) {
        file << point.x << " " << point.y << " " << point.z << "\n";
        if (!file.good()) {
            return false;
        }
    }
    
    return true;
}

bool PlyIO::isValidPropertySet(const std::vector<std::string>& properties) {
    bool has_x = false, has_y = false, has_z = false;
    
    for (const auto& prop : properties) {
        if (prop.find(" x") != std::string::npos || prop.find(" x\n") != std::string::npos) {
            has_x = true;
        }
        if (prop.find(" y") != std::string::npos || prop.find(" y\n") != std::string::npos) {
            has_y = true;
        }
        if (prop.find(" z") != std::string::npos || prop.find(" z\n") != std::string::npos) {
            has_z = true;
        }
    }
    
    return has_x && has_y && has_z;
}

bool PlyIO::isBinaryFormat(const std::string& format) {
    return format == "binary_little_endian" || format == "binary_big_endian";
}

bool PlyIO::readBinaryData(std::ifstream& file, PointCloud& cloud, const PlyHeader& header) {
    cloud.points.reserve(header.vertex_count);
    
    // Read binary vertex data
    for (int i = 0; i < header.vertex_count; ++i) {
        float x, y, z;
        
        // Read 3 floats (12 bytes) in little endian format
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        
        if (!file.good()) {
            std::cerr << "Failed to read binary vertex data at vertex " << i << std::endl;
            return false;
        }
        
        cloud.points.emplace_back(x, y, z);
    }
    
    return true;
}

} // namespace pointcloud_compressor