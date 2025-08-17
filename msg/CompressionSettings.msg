# Compression settings used for point cloud compression

# Voxel size used for voxelization (meters)
float32 voxel_size

# Block size for pattern extraction (e.g., 8 for 8x8x8 blocks)
uint32 block_size

# Whether 8-bit indices are used (true) or 16-bit indices (false)
bool use_8bit_indices

# Compression algorithm version
string algorithm_version

# Additional parameters as key-value pairs
string[] parameter_names
string[] parameter_values