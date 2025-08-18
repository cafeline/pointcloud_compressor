# Pattern dictionary containing unique voxel block patterns

# Header for timestamp and frame_id
std_msgs/Header header

# Compression settings
float64 voxel_size
uint32 block_size
bool use_8bit_indices
uint32 min_points_threshold

# Voxel grid dimensions
geometry_msgs/Vector3 voxel_grid_dimensions
geometry_msgs/Vector3 voxel_grid_origin

# Block information
uint32 total_blocks
uint32 blocks_x
uint32 blocks_y
uint32 blocks_z
uint8[] block_indices

# Number of unique patterns in the dictionary
uint32 num_patterns

# Size of each pattern in bytes
uint32 pattern_size_bytes

# Flattened dictionary data containing all unique patterns
uint8[] dictionary_data

# CRC32 checksum for data integrity
uint32 checksum

# Statistics
float32 compression_ratio
uint32 original_point_count
uint32 compressed_data_size