// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_FILE_FORMAT_DETECTOR_HPP
#define VQ_OCCUPANCY_COMPRESSOR_FILE_FORMAT_DETECTOR_HPP

#include <string>

namespace vq_occupancy_compressor {


enum class FileFormat {
    UNKNOWN,
    PCD,
    PLY
};


class FileFormatDetector {
public:
    
    static FileFormat detectFormat(const std::string& filename);
    
    
    static FileFormat detectByExtension(const std::string& filename);
    
    
    static FileFormat detectByContent(const std::string& filename);
    
    
    static std::string formatToString(FileFormat format);
    
    
    static bool isSupportedFormat(FileFormat format);
    
    
    static std::string getFileExtension(const std::string& filename);

private:
    
    static std::string normalizeExtension(const std::string& extension);
};

} 

#endif 