#include "pointcloud_compressor/io/PcdIO.hpp"
#include <filesystem>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace pointcloud_compressor {

bool PcdIO::readPcdFile(const std::string& filename, PointCloud& cloud) {
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
    if (!parseHeaderInternal(file, header)) {
        std::cerr << "Error: Failed to parse PCD header" << std::endl;
        return false;
    }
    
    // Read data based on type
    if (header.data_type == "ascii") {
        return readAsciiData(file, cloud, header);
    } else {
        std::cerr << "Error: Binary PCD format not yet supported" << std::endl;
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

} // namespace pointcloud_compressor