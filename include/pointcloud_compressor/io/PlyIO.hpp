#ifndef POINTCLOUD_COMPRESSOR_PLY_IO_HPP
#define POINTCLOUD_COMPRESSOR_PLY_IO_HPP

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "PcdIO.hpp"  // For Point3D and PointCloud

namespace pointcloud_compressor {

// PLY file header structure
struct PlyHeader {
    std::string format;         // ascii or binary
    std::string version;        // version number
    int vertex_count;           // number of vertices
    std::vector<std::string> comments;  // comment lines
    std::vector<std::string> properties; // property definitions
    std::vector<std::string> property_names; // parsed property names (e.g., x,y,z,...)
    
    PlyHeader() : format("ascii"), version("1.0"), vertex_count(0) {}
};

// PLY file I/O class
class PlyIO {
public:
    // Read PLY file into point cloud
    static bool readPlyFile(const std::string& filename, PointCloud& cloud);
    
    // Write point cloud to PLY file
    static bool writePlyFile(const std::string& filename, const PointCloud& cloud);
    
    // Parse PLY header
    static bool parseHeader(const std::string& filename, PlyHeader& header);
    
    // Validate PLY file format
    static bool validateFormat(const std::string& filename);
    
private:
    // Internal helper functions
    static bool parseHeaderInternal(std::ifstream& file, PlyHeader& header);
    static bool readAsciiData(std::ifstream& file, PointCloud& cloud, const PlyHeader& header);
    static bool readBinaryData(std::ifstream& file, PointCloud& cloud, const PlyHeader& header);
    static bool readBinaryDataMmap(const std::string& filename, PointCloud& cloud, const PlyHeader& header, size_t header_size);
    static bool writeHeader(std::ofstream& file, const PointCloud& cloud);
    static bool writeAsciiData(std::ofstream& file, const PointCloud& cloud);
    static bool isValidPropertySet(const std::vector<std::string>& properties);
    static bool isBinaryFormat(const std::string& format);
};

} // namespace pointcloud_compressor

#endif // POINTCLOUD_COMPRESSOR_PLY_IO_HPP
