# Compressed point cloud message

# Standard ROS header
std_msgs/Header header

# Original point cloud metadata
geometry_msgs/Vector3 original_min_bounds    # Minimum bounds of original point cloud
geometry_msgs/Vector3 original_max_bounds    # Maximum bounds of original point cloud
uint32 original_point_count                  # Number of points in original cloud
uint32 original_data_size                    # Size of original data in bytes

# Voxel grid information
geometry_msgs/Vector3 voxel_grid_dimensions  # Voxel grid dimensions (x, y, z)
geometry_msgs/Vector3 voxel_grid_origin      # Origin of voxel grid

# Block information
uint32 total_blocks                          # Total number of blocks
uint32 blocks_x                              # Number of blocks in x direction
uint32 blocks_y                              # Number of blocks in y direction
uint32 blocks_z                              # Number of blocks in z direction

# Compression settings
CompressionSettings compression_settings

# Pattern dictionary
PatternDictionary pattern_dictionary

# Block indices (one index per block, referencing dictionary patterns)
uint16[] block_indices

# Compression statistics
float32 compression_ratio                    # Compressed size / original size
uint32 compressed_data_size                  # Total size of compressed data in bytes
float64 compression_time_seconds             # Time taken for compression

# Quality metrics
float32 reconstruction_error                 # Average reconstruction error (if available)
uint32 unique_patterns_count                 # Number of unique patterns found