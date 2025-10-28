// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/io/PcdIO.hpp"
#include <filesystem>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace vq_occupancy_compressor {

bool PcdIO::readPcdFile(const std::string& filename, PointCloud& cloud) {
    auto t0 = std::chrono::high_resolution_clock::now();
    // Clear existing data
    cloud.clear();
    
    // Check if file exists
    if (!std::filesystem::exists(filename)) {
        std::cerr << "Error: File does not exist: " << filename << std::endl;
        return false;
    }
    
    // Open file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file: " << filename << std::endl;
        return false;
    }
    
    // Check if file is empty
    file.seekg(0, std::ios::end);
    if (file.tellg() == 0) {
        std::cerr << "Error: File is empty: " << filename << std::endl;
        return false;
    }
    file.seekg(0, std::ios::beg);
    
    // Parse header
    PcdHeader header;
    auto th0 = std::chrono::high_resolution_clock::now();
    if (!parseHeaderInternal(file, header)) {
        std::cerr << "Error: Failed to parse PCD header" << std::endl;
        return false;
    }
    auto th1 = std::chrono::high_resolution_clock::now();
    
    // Read data based on type
    if (header.data_type == "ascii") {
        bool ok = readAsciiData(file, cloud, header);
        auto t1 = std::chrono::high_resolution_clock::now();
        double header_ms = std::chrono::duration_cast<std::chrono::microseconds>(th1 - th0).count() / 1000.0;
        double total_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
        std::cout << "[PROFILE][PcdIO] data=ascii, header=" << header_ms << " ms, total=" << total_ms
                  << " ms, points=" << header.points << std::endl;
        return ok;
    } else if (header.data_type == "binary" || header.data_type == "binary_compressed") {
        // For binary data, we need to reopen the file in binary mode
        std::streampos data_start = file.tellg();
        file.close();
        
        // Reopen in binary mode
        std::ifstream binary_file(filename, std::ios::binary);
        if (!binary_file.is_open()) {
            std::cerr << "Error: Cannot reopen file in binary mode: " << filename << std::endl;
            return false;
        }
        
        // Skip to data section
        binary_file.seekg(data_start);
        
        bool ok = readBinaryData(binary_file, cloud, header);
        auto t1 = std::chrono::high_resolution_clock::now();
        double header_ms = std::chrono::duration_cast<std::chrono::microseconds>(th1 - th0).count() / 1000.0;
        double total_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
        std::cout << "[PROFILE][PcdIO] data=" << header.data_type
                  << ", header=" << header_ms << " ms, total=" << total_ms
                  << " ms, points=" << header.points << std::endl;
        return ok;
    } else {
        std::cerr << "Error: Unknown PCD data type: " << header.data_type << std::endl;
        return false;
    }
}

bool PcdIO::writePcdFile(const std::string& filename, const PointCloud& cloud) {
    // Check if directory exists
    std::filesystem::path filepath(filename);
    std::filesystem::path dir = filepath.parent_path();
    
    if (!dir.empty() && !std::filesystem::exists(dir)) {
        std::cerr << "Error: Directory does not exist: " << dir << std::endl;
        return false;
    }
    
    // Open file for writing
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create file: " << filename << std::endl;
        return false;
    }
    
    // Write header
    if (!writeHeader(file, cloud)) {
        std::cerr << "Error: Failed to write PCD header" << std::endl;
        return false;
    }
    
    // Write data
    if (!writeAsciiData(file, cloud)) {
        std::cerr << "Error: Failed to write PCD data" << std::endl;
        return false;
    }
    
    file.close();
    return true;
}

bool PcdIO::parseHeader(const std::string& filename, PcdHeader& header) {
    // Check if file exists
    if (!std::filesystem::exists(filename)) {
        return false;
    }
    
    // Open file
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    return parseHeaderInternal(file, header);
}

bool PcdIO::parseHeaderInternal(std::ifstream& file, PcdHeader& header) {
    std::string line;
    bool found_data = false;
    
    while (std::getline(file, line)) {
        // Skip comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream iss(line);
        std::string key;
        iss >> key;
        
        if (key == "VERSION") {
            iss >> header.version;
        } else if (key == "FIELDS") {
            std::string field;
            while (iss >> field) {
                header.fields.push_back(field);
            }
        } else if (key == "WIDTH") {
            iss >> header.width;
        } else if (key == "HEIGHT") {
            iss >> header.height;
        } else if (key == "POINTS") {
            iss >> header.points;
        } else if (key == "DATA") {
            iss >> header.data_type;
            found_data = true;
            break;  // DATA should be the last header field
        }
    }
    
    return found_data && header.points > 0;
}

bool PcdIO::readAsciiData(std::ifstream& file, PointCloud& cloud, const PcdHeader& header) {
    cloud.points.reserve(header.points);
    
    std::string line;
    int points_read = 0;
    
    while (std::getline(file, line) && points_read < header.points) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        float x, y, z;
        
        if (iss >> x >> y >> z) {
            cloud.points.emplace_back(x, y, z);
            points_read++;
        }
    }
    
    return points_read == header.points;
}

bool PcdIO::writeHeader(std::ofstream& file, const PointCloud& cloud) {
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << cloud.points.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << cloud.points.size() << "\n";
    file << "DATA ascii\n";
    
    return file.good();
}

bool PcdIO::writeAsciiData(std::ofstream& file, const PointCloud& cloud) {
    // Set precision for floating point output
    file << std::fixed << std::setprecision(6);
    
    for (const auto& point : cloud.points) {
        file << point.x << " " << point.y << " " << point.z << "\n";
    }
    
    return file.good();
}

bool PcdIO::readBinaryData(std::ifstream& file, PointCloud& cloud, const PcdHeader& header) {
    // Reserve space for points
    cloud.points.reserve(header.points);
    
    // Check for fields to determine data layout
    bool has_xyz = false;
    int x_idx = -1, y_idx = -1, z_idx = -1;
    
    for (size_t i = 0; i < header.fields.size(); ++i) {
        if (header.fields[i] == "x" || header.fields[i] == "X") x_idx = i;
        else if (header.fields[i] == "y" || header.fields[i] == "Y") y_idx = i;
        else if (header.fields[i] == "z" || header.fields[i] == "Z") z_idx = i;
    }
    
    if (x_idx >= 0 && y_idx >= 0 && z_idx >= 0) {
        has_xyz = true;
    }
    
    if (!has_xyz) {
        // If no field names, assume first 3 fields are x, y, z
        if (header.fields.size() >= 3) {
            x_idx = 0;
            y_idx = 1;
            z_idx = 2;
            has_xyz = true;
        } else {
            std::cerr << "Error: PCD file does not contain x, y, z fields" << std::endl;
            return false;
        }
    }
    
    // For binary format, data is typically stored as a continuous block
    // Common format: each point is stored as consecutive floats
    int fields_per_point = header.fields.empty() ? 3 : header.fields.size();
    
    // Read binary data
    for (int i = 0; i < header.points; ++i) {
        std::vector<float> point_data(fields_per_point);
        
        // Read all fields for this point
        file.read(reinterpret_cast<char*>(point_data.data()), fields_per_point * sizeof(float));
        
        if (file.eof() && i == header.points - 1) {
            // EOF is ok for the last point
            break;
        }
        
        if (!file.good()) {
            std::cerr << "Error: Failed to read binary data at point " << i 
                      << " (read " << cloud.points.size() << " points so far)" << std::endl;
            // Return true if we read at least some points
            return !cloud.points.empty();
        }
        
        // Extract x, y, z values
        Point3D point;
        point.x = point_data[x_idx];
        point.y = point_data[y_idx];
        point.z = point_data[z_idx];
        
        // Skip invalid points (NaN or Inf)
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            cloud.points.push_back(point);
        }
    }
    
    // Log actual points read
    if (cloud.points.size() != static_cast<size_t>(header.points)) {
        std::cerr << "Info: Expected " << header.points << " points, read " 
                  << cloud.points.size() << " valid points" << std::endl;
    }
    
    return !cloud.points.empty();
}

} // namespace vq_occupancy_compressor
