#include <gtest/gtest.h>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include "pointcloud_compressor/core/BitVoxelGrid.hpp"

using namespace pointcloud_compressor;

// Test fixture
class BitVoxelGridTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }
    
    void TearDown() override {
        // Cleanup code if needed
    }
};

// Test basic bit operations
TEST_F(BitVoxelGridTest, BasicBitOperations) {
    BitVoxelGrid grid(10, 10, 10);
    
    // Test initial state
    EXPECT_FALSE(grid.getBit(0));
    EXPECT_FALSE(grid.getBit(999));
    
    // Test setBit
    grid.setBit(0);
    EXPECT_TRUE(grid.getBit(0));
    
    grid.setBit(999);
    EXPECT_TRUE(grid.getBit(999));
    
    // Test clearBit
    grid.clearBit(0);
    EXPECT_FALSE(grid.getBit(0));
    
    // Test boundary
    grid.setBit(1000); // Out of bounds
    EXPECT_FALSE(grid.getBit(1000));
}

// Test 3D to 1D index conversion
TEST_F(BitVoxelGridTest, IndexConversion) {
    BitVoxelGrid grid(10, 20, 30);
    
    // Test valid indices
    EXPECT_EQ(grid.voxelIndex3Dto1D(0, 0, 0), 0);
    EXPECT_EQ(grid.voxelIndex3Dto1D(1, 0, 0), 1);
    EXPECT_EQ(grid.voxelIndex3Dto1D(0, 1, 0), 10);
    EXPECT_EQ(grid.voxelIndex3Dto1D(0, 0, 1), 200);
    EXPECT_EQ(grid.voxelIndex3Dto1D(9, 19, 29), 5999);
    
    // Test invalid indices
    EXPECT_EQ(grid.voxelIndex3Dto1D(-1, 0, 0), grid.getTotalVoxels());
    EXPECT_EQ(grid.voxelIndex3Dto1D(10, 0, 0), grid.getTotalVoxels());
    EXPECT_EQ(grid.voxelIndex3Dto1D(0, 20, 0), grid.getTotalVoxels());
}

// Test memory usage
TEST_F(BitVoxelGridTest, MemoryUsage) {
    // Small grid
    BitVoxelGrid small_grid(10, 10, 10);
    size_t small_memory = small_grid.getMemoryUsageBytes();
    EXPECT_EQ(small_memory, 16 * sizeof(uint64_t)); // 1000 bits = 16 uint64_t
    
    // Large grid (simulating 0.1m voxel size)
    BitVoxelGrid large_grid(3606, 2890, 687);
    size_t large_memory = large_grid.getMemoryUsageBytes();
    uint64_t total_voxels = static_cast<uint64_t>(3606) * 2890 * 687;
    size_t expected_words = (total_voxels + 63) / 64;
    EXPECT_EQ(large_memory, expected_words * sizeof(uint64_t));
    
    // Verify memory is much less than int array
    size_t int_array_size = total_voxels * sizeof(int);
    EXPECT_LT(large_memory, int_array_size / 30); // Should be at least 30x smaller
}

// Test counting occupied voxels
TEST_F(BitVoxelGridTest, CountOccupiedVoxels) {
    BitVoxelGrid grid(100, 100, 100);
    
    EXPECT_EQ(grid.countOccupiedVoxels(), 0);
    
    // Set some voxels
    for (int i = 0; i < 1000; i++) {
        grid.setBit(i * 10);
    }
    
    EXPECT_EQ(grid.countOccupiedVoxels(), 1000);
    
    // Clear some voxels
    for (int i = 0; i < 500; i++) {
        grid.clearBit(i * 10);
    }
    
    EXPECT_EQ(grid.countOccupiedVoxels(), 500);
}

// Test large-scale grid
TEST_F(BitVoxelGridTest, LargeScaleGrid) {
    // Simulate voxel_size = 0.1m scenario
    int grid_x = 3606;
    int grid_y = 2890;
    int grid_z = 687;
    
    BitVoxelGrid grid(grid_x, grid_y, grid_z);
    
    uint64_t total_voxels = static_cast<uint64_t>(grid_x) * grid_y * grid_z;
    EXPECT_EQ(grid.getTotalVoxels(), total_voxels);
    
    // Memory should be reasonable (< 1GB)
    size_t memory_mb = grid.getMemoryUsageBytes() / (1024 * 1024);
    EXPECT_LT(memory_mb, 1024); // Less than 1GB
    
    // Set a pattern of voxels
    std::unordered_map<uint64_t, bool> test_voxels;
    for (int i = 0; i < 100000; i++) {
        uint64_t idx = (i * 12345) % total_voxels;
        grid.setBit(idx);
        test_voxels[idx] = true;
    }
    
    // Verify the pattern
    for (const auto& [idx, _] : test_voxels) {
        EXPECT_TRUE(grid.getBit(idx));
    }
}

// Test edge cases
TEST_F(BitVoxelGridTest, EdgeCases) {
    // Minimum size grid
    BitVoxelGrid min_grid(1, 1, 1);
    EXPECT_EQ(min_grid.getTotalVoxels(), 1);
    min_grid.setBit(0);
    EXPECT_TRUE(min_grid.getBit(0));
    
    // Non-power-of-2 dimensions
    BitVoxelGrid odd_grid(17, 23, 31);
    uint64_t total = 17 * 23 * 31;
    EXPECT_EQ(odd_grid.getTotalVoxels(), total);
    
    // Test last voxel
    odd_grid.setBit(total - 1);
    EXPECT_TRUE(odd_grid.getBit(total - 1));
}

// Test performance with realistic workload
TEST_F(BitVoxelGridTest, RealisticWorkload) {
    // Simulate typical pointcloud processing
    BitVoxelGrid grid(500, 500, 100);
    
    // Simulate sparse occupancy (1% of voxels)
    std::vector<uint64_t> occupied_indices;
    for (int i = 0; i < 25000; i++) {
        uint64_t idx = rand() % grid.getTotalVoxels();
        grid.setBit(idx);
        occupied_indices.push_back(idx);
    }
    
    // Verify all set voxels are marked
    for (uint64_t idx : occupied_indices) {
        EXPECT_TRUE(grid.getBit(idx));
    }
    
    // Count should be reasonable (might have duplicates from random)
    size_t count = grid.countOccupiedVoxels();
    EXPECT_GT(count, 20000); // At least 80% unique
    EXPECT_LE(count, 25000); // At most all unique
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}