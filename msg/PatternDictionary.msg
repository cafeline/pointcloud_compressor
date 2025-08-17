# Pattern dictionary containing unique voxel block patterns

# Number of unique patterns in the dictionary
uint32 num_patterns

# Size of each pattern in bytes
uint32 pattern_size_bytes

# Flattened dictionary data containing all unique patterns
uint8[] dictionary_data

# CRC32 checksum for data integrity
uint32 checksum