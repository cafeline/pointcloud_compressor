#ifndef POINTCLOUD_COMPRESSOR_PCD_IO_HPP
#define POINTCLOUD_COMPRESSOR_PCD_IO_HPP

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

namespace pointcloud_compressor {

// 3D point structure
struct Point3D {
    float x, y, z;
    
    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// Point cloud container
struct PointCloud {
    std::vector<Point3D> points;
    
    void clear() { points.clear(); }
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
};

// PCD file header structure
struct PcdHeader {
    std::string version;
    int width;
    int height;
    int points;
    std::string data_type;  // ascii or binary
    std::vector<std::string> fields;
    
    PcdHeader() : width(0), height(0), points(0), data_type("ascii") {}
};

// PCD file I/O class
class PcdIO {
public:
    // Read PCD file into point cloud
    static bool readPcdFile(const std::string& filename, PointCloud& cloud);
    
    // Write point cloud to PCD file
    static bool writePcdFile(const std::string& filename, const PointCloud& cloud);
    
    // Parse PCD header
    static bool parseHeader(const std::string& filename, PcdHeader& header);
    
private:
    // Internal helper functions
    static bool parseHeaderInternal(std::ifstream& file, PcdHeader& header);
    static bool readAsciiData(std::ifstream& file, PointCloud& cloud, const PcdHeader& header);
    static bool readBinaryData(std::ifstream& file, PointCloud& cloud, const PcdHeader& header);
    static bool writeHeader(std::ofstream& file, const PointCloud& cloud);
    static bool writeAsciiData(std::ofstream& file, const PointCloud& cloud);
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_PCD_IO_HPP