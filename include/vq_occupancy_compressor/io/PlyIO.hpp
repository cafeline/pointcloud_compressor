// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_PLY_IO_HPP
#define VQ_OCCUPANCY_COMPRESSOR_PLY_IO_HPP

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "PcdIO.hpp"  

namespace vq_occupancy_compressor {


struct PlyHeader {
    std::string format;         
    std::string version;        
    int vertex_count;           
    std::vector<std::string> comments;  
    std::vector<std::string> properties; 
    std::vector<std::string> property_names; 
    
    PlyHeader() : format("ascii"), version("1.0"), vertex_count(0) {}
};


class PlyIO {
public:
    
    static bool readPlyFile(const std::string& filename, PointCloud& cloud);
    
    
    static bool writePlyFile(const std::string& filename, const PointCloud& cloud);
    
    
    static bool parseHeader(const std::string& filename, PlyHeader& header);
    
    
    static bool validateFormat(const std::string& filename);
    
private:
    
    static bool parseHeaderInternal(std::ifstream& file, PlyHeader& header);
    static bool readAsciiData(std::ifstream& file, PointCloud& cloud, const PlyHeader& header);
    static bool readBinaryData(std::ifstream& file, PointCloud& cloud, const PlyHeader& header);
    static bool readBinaryDataMmap(const std::string& filename, PointCloud& cloud, const PlyHeader& header, size_t header_size);
    static bool writeHeader(std::ofstream& file, const PointCloud& cloud);
    static bool writeAsciiData(std::ofstream& file, const PointCloud& cloud);
    static bool isValidPropertySet(const std::vector<std::string>& properties);
    static bool isBinaryFormat(const std::string& format);
};

} 

#endif 
