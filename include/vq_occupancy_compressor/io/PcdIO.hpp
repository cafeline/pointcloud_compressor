// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_PCD_IO_HPP
#define VQ_OCCUPANCY_COMPRESSOR_PCD_IO_HPP

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

namespace vq_occupancy_compressor {


struct Point3D {
    float x, y, z;
    
    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};


struct PointCloud {
    std::vector<Point3D> points;
    
    void clear() { points.clear(); }
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
};


struct PcdHeader {
    std::string version;
    int width;
    int height;
    int points;
    std::string data_type;  
    std::vector<std::string> fields;
    
    PcdHeader() : width(0), height(0), points(0), data_type("ascii") {}
};


class PcdIO {
public:
    
    static bool readPcdFile(const std::string& filename, PointCloud& cloud);
    
    
    static bool writePcdFile(const std::string& filename, const PointCloud& cloud);
    
    
    static bool parseHeader(const std::string& filename, PcdHeader& header);
    
private:
    
    static bool parseHeaderInternal(std::ifstream& file, PcdHeader& header);
    static bool readAsciiData(std::ifstream& file, PointCloud& cloud, const PcdHeader& header);
    static bool readBinaryData(std::ifstream& file, PointCloud& cloud, const PcdHeader& header);
    static bool writeHeader(std::ofstream& file, const PointCloud& cloud);
    static bool writeAsciiData(std::ofstream& file, const PointCloud& cloud);
};

} 

#endif 