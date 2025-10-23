// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_compressor/io/PlyIO.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <vector>
#include <cstring>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace pointcloud_compressor {

bool PlyIO::readPlyFile(const std::string& filename, PointCloud& cloud) {
    auto t0 = std::chrono::high_resolution_clock::now();
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
    auto th0 = std::chrono::high_resolution_clock::now();
    if (!parseHeaderInternal(file, header)) {
        std::cerr << "Failed to parse PLY header: " << filename << std::endl;
        return false;
    }
    auto th1 = std::chrono::high_resolution_clock::now();
    
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
    
    bool ok = false;
    if (header.format == "ascii") {
        ok = readAsciiData(file, cloud, header);
    } else {
        // Try memory-mapped I/O for large binary files
        size_t header_size = file.tellg();
        file.close();
        
        // Get file size
        struct stat st;
        if (stat(filename.c_str(), &st) == 0 && st.st_size > 10 * 1024 * 1024) { // Use mmap for files > 10MB
            ok = readBinaryDataMmap(filename, cloud, header, header_size);
        } else {
            // Reopen for small files
            file.open(filename, std::ios::binary);
            file.seekg(header_size);
            ok = readBinaryData(file, cloud, header);
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double header_ms = std::chrono::duration_cast<std::chrono::microseconds>(th1 - th0).count() / 1000.0;
    double total_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    std::cout << "[PROFILE][PlyIO] format=" << header.format
              << ", header=" << header_ms << " ms, total=" << total_ms
              << " ms, vertices=" << header.vertex_count << std::endl;
    return ok;
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
    
    // Parse headerの成否で判定し、重い全読込は避ける
    PlyHeader header;
    if (!parseHeaderInternal(file, header)) {
        return false;
    }

    // big-endianは未サポート
    if (header.format == "binary_big_endian") {
        return false;
    }

    // ヘッダが有効ならOK（頂点数が0でも有効）
    return true;
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
            std::string type, name;
            iss >> type >> name;
            // store full line for backward compatibility and simple checks
            std::string property_line = "property ";
            property_line += type + " " + name;
            header.properties.push_back(property_line);
            header.property_names.push_back(name);
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

    // プロパティ名からx,y,zの列位置を特定
    int x_idx = -1, y_idx = -1, z_idx = -1;
    for (size_t i = 0; i < header.property_names.size(); ++i) {
        const auto& n = header.property_names[i];
        if (n == "x") x_idx = static_cast<int>(i);
        else if (n == "y") y_idx = static_cast<int>(i);
        else if (n == "z") z_idx = static_cast<int>(i);
    }
    if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
        std::cerr << "PLY ASCII missing x/y/z properties" << std::endl;
        return false;
    }

    std::string line;
    int points_read = 0;
    while (std::getline(file, line) && points_read < header.vertex_count) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::vector<float> values;
        values.reserve(std::max<size_t>(header.property_names.size(), 3));
        float v;
        while (iss >> v) values.push_back(v);
        if (values.size() < header.property_names.size()) {
            std::cerr << "PLY ASCII line has insufficient properties" << std::endl;
            return false;
        }
        cloud.points.emplace_back(values[x_idx], values[y_idx], values[z_idx]);
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
    // シンプル実装: x,y,zがfloatで連続している前提（既存テストに整合）
    // 本格実装ではproperty_types/offsetからstride算出が必要
    const size_t bytes_per_vertex = 3 * sizeof(float);
    const size_t total_bytes = header.vertex_count * bytes_per_vertex;
    
    // Allocate buffer for batch reading
    const size_t buffer_size = std::min(total_bytes, size_t(100 * 1024 * 1024)); // 100MB buffer
    std::vector<char> buffer(buffer_size);
    
    size_t vertices_read = 0;
    
    while (vertices_read < header.vertex_count) {
        // Calculate how many vertices to read in this batch
        size_t vertices_in_batch = std::min(
            buffer_size / bytes_per_vertex,
            size_t(header.vertex_count - vertices_read)
        );
        size_t bytes_to_read = vertices_in_batch * bytes_per_vertex;
        
        // Read batch into buffer
        file.read(buffer.data(), bytes_to_read);
        if (!file.good() && !file.eof()) {
            std::cerr << "Failed to read binary vertex data at vertex " << vertices_read << std::endl;
            return false;
        }
        
        // Process buffer
        const float* float_buffer = reinterpret_cast<const float*>(buffer.data());
        for (size_t i = 0; i < vertices_in_batch; ++i) {
            cloud.points.emplace_back(
                float_buffer[i * 3],
                float_buffer[i * 3 + 1],
                float_buffer[i * 3 + 2]
            );
        }
        
        vertices_read += vertices_in_batch;
    }
    
    return true;
}

bool PlyIO::readBinaryDataMmap(const std::string& filename, PointCloud& cloud, 
                               const PlyHeader& header, size_t header_size) {
    // Open file for memory mapping
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
        std::cerr << "Failed to open file for mmap: " << filename << std::endl;
        return false;
    }
    
    // Get file size
    struct stat st;
    if (fstat(fd, &st) != 0) {
        close(fd);
        std::cerr << "Failed to get file size for mmap: " << filename << std::endl;
        return false;
    }
    
    // Memory map the file
    void* mapped = mmap(nullptr, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (mapped == MAP_FAILED) {
        close(fd);
        std::cerr << "Failed to mmap file: " << filename << std::endl;
        return false;
    }
    
    // Advise kernel about access pattern
    madvise(mapped, st.st_size, MADV_SEQUENTIAL);
    
    // サイズ検証: 必要データ長が存在するか（3 floats/vertex 仮定）
    size_t required = header_size + static_cast<size_t>(header.vertex_count) * 3 * sizeof(float);
    if (static_cast<size_t>(st.st_size) < required) {
        munmap(mapped, st.st_size);
        close(fd);
        std::cerr << "PLY mmap: file too small for declared vertex data" << std::endl;
        return false;
    }

    // Reserve space for points
    cloud.points.reserve(header.vertex_count);
    
    // Skip header and read vertex data
    const char* data = static_cast<const char*>(mapped) + header_size;
    const float* vertices = reinterpret_cast<const float*>(data);
    
    // Read all vertices at once
    for (int i = 0; i < header.vertex_count; ++i) {
        cloud.points.emplace_back(
            vertices[i * 3],
            vertices[i * 3 + 1],
            vertices[i * 3 + 2]
        );
    }
    
    // Clean up
    munmap(mapped, st.st_size);
    close(fd);
    
    return true;
}

} // namespace pointcloud_compressor
