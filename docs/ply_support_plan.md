# PLY File Support Implementation Plan

## Requirements Analysis

### Current System
- **Point3D**: Simple x,y,z float structure
- **PointCloud**: Vector of Point3D
- **PcdIO**: Static methods for PCD file I/O
- **Supported format**: PCD ASCII only

### PLY Format Requirements
- **Header structure**: ply, format, element, property definitions
- **Data format**: ASCII x y z coordinates
- **Features to support**: 
  - ASCII format PLY files
  - Vertex element with x,y,z properties
  - Comments and metadata

### Integration Requirements
- **File format detection**: Auto-detect PCD vs PLY by extension/header
- **Unified interface**: Single method to load either format
- **Backward compatibility**: Existing PCD functionality unchanged
- **Error handling**: Clear error messages for unsupported PLY features

## Test-Driven Development Plan

### Phase 1: PLY File Reading Tests
1. **test_ply_basic_reading**: Read simple PLY file with 8 vertices
2. **test_ply_header_parsing**: Parse PLY header structure correctly
3. **test_ply_data_parsing**: Parse vertex data into Point3D objects
4. **test_ply_empty_file**: Handle empty PLY files
5. **test_ply_invalid_format**: Handle malformed PLY files

### Phase 2: File Format Detection Tests  
1. **test_file_format_detection_by_extension**: .pcd vs .ply detection
2. **test_file_format_detection_by_header**: Content-based detection
3. **test_mixed_extension_content**: Wrong extension with correct content

### Phase 3: Unified Interface Tests
1. **test_unified_loader_pcd**: Load PCD through unified interface
2. **test_unified_loader_ply**: Load PLY through unified interface
3. **test_unified_loader_auto_detect**: Auto-detect and load both formats
4. **test_unified_loader_error_handling**: Handle various error conditions

### Phase 4: Integration Tests
1. **test_compression_with_ply**: Full compression pipeline with PLY input
2. **test_visualization_with_ply**: Full visualization pipeline with PLY input
3. **test_launch_file_ply_parameter**: Launch file accepts PLY files

## Implementation Architecture

### New Components
- **PlyIO class**: Static methods for PLY file operations
- **PointCloudIO class**: Unified interface for both PCD and PLY
- **FileFormatDetector**: Utility for format detection

### File Structure
```
include/pointcloud_compressor/io/
├── PcdIO.hpp (existing)
├── PlyIO.hpp (new)
├── PointCloudIO.hpp (new - unified interface)
└── FileFormatDetector.hpp (new)

src/io/
├── PcdIO.cpp (existing)
├── PlyIO.cpp (new)
├── PointCloudIO.cpp (new)
└── FileFormatDetector.cpp (new)

test/
├── test_pcd_io.cpp (existing)
├── test_ply_io.cpp (new)
├── test_pointcloud_io.cpp (new)
└── test_data/
    ├── sample.pcd (existing)
    ├── sample.ply (new)
    ├── cube.ply (new)
    └── invalid.ply (new)
```

### API Design
```cpp
// Unified interface
class PointCloudIO {
public:
    static bool loadPointCloud(const std::string& filename, PointCloud& cloud);
    static bool savePointCloud(const std::string& filename, const PointCloud& cloud);
    static FileFormat detectFormat(const std::string& filename);
};

// PLY specific
class PlyIO {
public:
    static bool readPlyFile(const std::string& filename, PointCloud& cloud);
    static bool writePlyFile(const std::string& filename, const PointCloud& cloud);
    static bool parseHeader(const std::string& filename, PlyHeader& header);
};
```

## Test Data Requirements
- **Simple PLY**: 8-vertex cube
- **Large PLY**: 1000+ vertices for performance testing  
- **Comments PLY**: File with comment lines
- **Invalid PLY**: Malformed files for error testing
- **Mixed format**: PCD file with .ply extension

## Success Criteria
1. All existing PCD tests continue to pass
2. PLY files load correctly into Point3D structures
3. File format auto-detection works reliably
4. Compression pipeline works with PLY input
5. Launch files accept both PCD and PLY parameters
6. Performance comparable to PCD loading
7. Clear error messages for unsupported features

## Implementation Order
1. Create test files and data
2. Implement PLY reading tests (Red phase)
3. Implement PlyIO class (Green phase)
4. Add file format detection tests
5. Implement unified PointCloudIO interface
6. Integrate with compression pipeline
7. Update launch file parameters
8. Run integration tests