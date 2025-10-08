#include <gtest/gtest.h>
#include <random>
#include "pointcloud_compressor/core/VoxelProcessor.hpp"
#include "pointcloud_compressor/io/PcdIO.hpp"

using namespace pointcloud_compressor;

class VoxelProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default voxel size and block size
        voxel_size = 0.01f;
        block_size = 8;
    }
    
    // Helper function to create a simple point cloud
    PointCloud createSimplePointCloud() {
        PointCloud cloud;
        // Create a 3x3x3 grid of points
        for (float x = 0; x < 0.03f; x += 0.01f) {
            for (float y = 0; y < 0.03f; y += 0.01f) {
                for (float z = 0; z < 0.03f; z += 0.01f) {
                    cloud.points.emplace_back(x, y, z);
                }
            }
        }
        return cloud;
    }
    
    // Helper function to create a random point cloud
    PointCloud createRandomPointCloud(size_t num_points, float range = 1.0f) {
        PointCloud cloud;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-range, range);
        
        for (size_t i = 0; i < num_points; ++i) {
            cloud.points.emplace_back(dis(gen), dis(gen), dis(gen));
        }
        return cloud;
    }
    
    float voxel_size;
    int block_size;
};

// Test voxelization of point cloud
TEST_F(VoxelProcessorTest, VoxelizePointCloud) {
    VoxelProcessor processor(voxel_size, block_size);
    PointCloud cloud = createSimplePointCloud();
    
    VoxelGrid grid;
    bool result = processor.voxelizePointCloud(cloud, grid);
    
    EXPECT_TRUE(result);
    EXPECT_GT(grid.getOccupiedVoxelCount(), 0);
    
    // Check dimensions
    auto dims = grid.getDimensions();
    EXPECT_EQ(dims.x, 3);  // 3 voxels in x
    EXPECT_EQ(dims.y, 3);  // 3 voxels in y
    EXPECT_EQ(dims.z, 3);  // 3 voxels in z
}

TEST_F(VoxelProcessorTest, VoxelizePointCloudWithMargin) {
    VoxelProcessor processor(voxel_size, block_size);
    processor.setBoundingBoxMarginRatio(0.3f);
    PointCloud cloud = createSimplePointCloud();

    VoxelGrid grid;
    bool result = processor.voxelizePointCloud(cloud, grid);

    EXPECT_TRUE(result);

    auto dims = grid.getDimensions();
    // Original extent was 0.02 -> 3 voxels. With 30% margin each side -> expanded diff 1.6x.
    EXPECT_EQ(dims.x, 5);
    EXPECT_EQ(dims.y, 5);
    EXPECT_EQ(dims.z, 5);

    float ox, oy, oz;
    grid.getOrigin(ox, oy, oz);
    EXPECT_NEAR(ox, -0.006f, 1e-6f);
    EXPECT_NEAR(oy, -0.006f, 1e-6f);
    EXPECT_NEAR(oz, -0.006f, 1e-6f);
}

// Test empty point cloud voxelization
TEST_F(VoxelProcessorTest, VoxelizeEmptyPointCloud) {
    VoxelProcessor processor(voxel_size, block_size);
    PointCloud cloud;  // Empty cloud
    
    VoxelGrid grid;
    bool result = processor.voxelizePointCloud(cloud, grid);
    
    EXPECT_FALSE(result);
    EXPECT_EQ(grid.getOccupiedVoxelCount(), 0);
}

// Test block division
TEST_F(VoxelProcessorTest, DivideIntoBlocks) {
    VoxelProcessor processor(voxel_size, block_size);
    PointCloud cloud = createSimplePointCloud();
    
    VoxelGrid grid;
    processor.voxelizePointCloud(cloud, grid);
    
    std::vector<VoxelBlock> blocks;
    bool result = processor.divideIntoBlocks(grid, blocks);
    
    EXPECT_TRUE(result);
    EXPECT_GT(blocks.size(), 0);
    
    // Each block should have the correct size
    for (const auto& block : blocks) {
        EXPECT_EQ(block.size, block_size);
    }
}

// Test block count calculation
TEST_F(VoxelProcessorTest, CalculateBlockCount) {
    VoxelProcessor processor(voxel_size, block_size);
    
    // Create a voxel grid with known dimensions
    VoxelGrid grid;
    grid.setDimensions(24, 16, 8);  // Should result in 3x2x1 blocks
    
    int x_blocks, y_blocks, z_blocks, total_blocks;
    processor.calculateBlockCount(grid, x_blocks, y_blocks, z_blocks, total_blocks);
    
    EXPECT_EQ(x_blocks, 3);
    EXPECT_EQ(y_blocks, 2);
    EXPECT_EQ(z_blocks, 1);
    EXPECT_EQ(total_blocks, 6);
}

// Test pattern extraction
TEST_F(VoxelProcessorTest, ExtractPatterns) {
    VoxelProcessor processor(voxel_size, block_size);
    
    // Create a simple voxel block
    VoxelBlock block(block_size);
    block.setVoxel(0, 0, 0, true);
    block.setVoxel(1, 1, 1, true);
    block.setVoxel(2, 2, 2, true);
    
    std::vector<uint8_t> pattern = processor.extractPattern(block);
    
    // Pattern size should be ceil(block_size^3 / 8) bytes
    int expected_bits = block_size * block_size * block_size;
    int expected_bytes = (expected_bits + 7) / 8;
    EXPECT_EQ(pattern.size(), expected_bytes);
    
    // Check that at least some bits are set
    bool has_set_bits = false;
    for (uint8_t byte : pattern) {
        if (byte != 0) {
            has_set_bits = true;
            break;
        }
    }
    EXPECT_TRUE(has_set_bits);
}

// Test voxel size optimization
TEST_F(VoxelProcessorTest, OptimizeVoxelSize) {
    VoxelProcessor processor(voxel_size, block_size);
    PointCloud cloud = createRandomPointCloud(1000);
    
    float optimal_size = processor.findOptimalVoxelSize(cloud, 0.005f, 0.05f, 0.005f);
    
    EXPECT_GE(optimal_size, 0.005f);
    EXPECT_LE(optimal_size, 0.05f);
}

// Test reconstruction from voxel grid
TEST_F(VoxelProcessorTest, ReconstructFromVoxelGrid) {
    VoxelProcessor processor(voxel_size, block_size);
    PointCloud original = createSimplePointCloud();
    
    // Voxelize
    VoxelGrid grid;
    processor.voxelizePointCloud(original, grid);
    
    // Reconstruct
    PointCloud reconstructed;
    bool result = processor.reconstructPointCloud(grid, reconstructed);
    
    EXPECT_TRUE(result);
    EXPECT_GT(reconstructed.points.size(), 0);
    
    // Check that reconstructed points are close to voxel centers
    for (const auto& point : reconstructed.points) {
        // Points should be at voxel centers
        float voxel_x = std::floor(point.x / voxel_size) * voxel_size + voxel_size / 2;
        float voxel_y = std::floor(point.y / voxel_size) * voxel_size + voxel_size / 2;
        float voxel_z = std::floor(point.z / voxel_size) * voxel_size + voxel_size / 2;
        
        EXPECT_NEAR(point.x, voxel_x, voxel_size);
        EXPECT_NEAR(point.y, voxel_y, voxel_size);
        EXPECT_NEAR(point.z, voxel_z, voxel_size);
    }
}

// Test pattern size calculation
TEST_F(VoxelProcessorTest, CalculatePatternSize) {
    VoxelProcessor processor(voxel_size, block_size);
    
    int pattern_bits, pattern_bytes;
    processor.calculatePatternSize(pattern_bits, pattern_bytes);
    
    int expected_bits = block_size * block_size * block_size;
    int expected_bytes = (expected_bits + 7) / 8;
    
    EXPECT_EQ(pattern_bits, expected_bits);
    EXPECT_EQ(pattern_bytes, expected_bytes);
}

// Test boundary handling
TEST_F(VoxelProcessorTest, HandleBoundaryBlocks) {
    VoxelProcessor processor(voxel_size, block_size);
    
    // Create a grid that doesn't divide evenly by block size
    VoxelGrid grid;
    grid.setDimensions(10, 10, 10);  // Not divisible by 8
    
    std::vector<VoxelBlock> blocks;
    bool result = processor.divideIntoBlocks(grid, blocks);
    
    EXPECT_TRUE(result);
    
    // Should have 2x2x2 = 8 blocks (with padding)
    int x_blocks = (10 + block_size - 1) / block_size;
    int y_blocks = (10 + block_size - 1) / block_size;
    int z_blocks = (10 + block_size - 1) / block_size;
    EXPECT_EQ(blocks.size(), x_blocks * y_blocks * z_blocks);
}

// Test voxel occupancy threshold - single point per voxel (should not be occupied with threshold > 1)
TEST_F(VoxelProcessorTest, VoxelOccupancyThresholdSinglePoint) {
    int min_points_threshold = 2;  // Require at least 2 points to mark voxel as occupied
    VoxelProcessor processor(voxel_size, block_size, min_points_threshold);
    
    // Create a point cloud with single points in different voxels
    PointCloud cloud;
    cloud.points.emplace_back(0.005f, 0.005f, 0.005f);  // Center of first voxel
    cloud.points.emplace_back(0.015f, 0.005f, 0.005f);  // Center of second voxel
    cloud.points.emplace_back(0.005f, 0.015f, 0.005f);  // Center of third voxel
    
    VoxelGrid grid;
    bool result = processor.voxelizePointCloud(cloud, grid);
    
    EXPECT_TRUE(result);
    // With threshold=2, no voxels should be marked as occupied since each has only 1 point
    EXPECT_EQ(grid.getOccupiedVoxelCount(), 0);
}

// Test voxel occupancy threshold - multiple points per voxel (should be occupied with threshold met)
TEST_F(VoxelProcessorTest, VoxelOccupancyThresholdMultiplePoints) {
    int min_points_threshold = 3;  // Require at least 3 points to mark voxel as occupied
    VoxelProcessor processor(voxel_size, block_size, min_points_threshold);
    
    // Create a point cloud with multiple points in the same voxel
    PointCloud cloud;
    // 4 points in the same voxel (first voxel: 0.0-0.01 range)
    cloud.points.emplace_back(0.001f, 0.001f, 0.001f);
    cloud.points.emplace_back(0.002f, 0.002f, 0.002f);
    cloud.points.emplace_back(0.003f, 0.003f, 0.003f);
    cloud.points.emplace_back(0.004f, 0.004f, 0.004f);
    
    // 2 points in another voxel (second voxel: 0.01-0.02 range)
    cloud.points.emplace_back(0.011f, 0.001f, 0.001f);
    cloud.points.emplace_back(0.012f, 0.002f, 0.002f);
    
    VoxelGrid grid;
    bool result = processor.voxelizePointCloud(cloud, grid);
    
    EXPECT_TRUE(result);
    // With threshold=3, only the first voxel (4 points) should be occupied
    // The second voxel (2 points) should not be occupied
    EXPECT_EQ(grid.getOccupiedVoxelCount(), 1);
}

// Test voxel occupancy threshold - default behavior (threshold = 1)
TEST_F(VoxelProcessorTest, VoxelOccupancyDefaultThreshold) {
    // Default threshold should be 1 (any point marks voxel as occupied)
    VoxelProcessor processor(voxel_size, block_size);  // No threshold specified, should default to 1
    
    PointCloud cloud;
    cloud.points.emplace_back(0.005f, 0.005f, 0.005f);  // Single point
    
    VoxelGrid grid;
    bool result = processor.voxelizePointCloud(cloud, grid);
    
    EXPECT_TRUE(result);
    // With default threshold=1, the voxel should be occupied
    EXPECT_EQ(grid.getOccupiedVoxelCount(), 1);
}

// Test voxel occupancy threshold - zero threshold (should behave like threshold=1)
TEST_F(VoxelProcessorTest, VoxelOccupancyZeroThreshold) {
    int min_points_threshold = 0;  // Zero threshold should behave like 1
    VoxelProcessor processor(voxel_size, block_size, min_points_threshold);
    
    PointCloud cloud;
    cloud.points.emplace_back(0.005f, 0.005f, 0.005f);  // Single point
    
    VoxelGrid grid;
    bool result = processor.voxelizePointCloud(cloud, grid);
    
    EXPECT_TRUE(result);
    // With threshold=0 (treated as 1), the voxel should be occupied
    EXPECT_EQ(grid.getOccupiedVoxelCount(), 1);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
