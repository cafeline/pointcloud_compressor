#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "pointcloud_compressor/io/PlyIO.hpp"

using namespace pointcloud_compressor;

class PlyPropertyOrderTest : public ::testing::Test {
protected:
    std::string tmp_file;
    void SetUp() override {
        tmp_file = std::filesystem::temp_directory_path() / "ply_prop_order_test.ply";
        std::ofstream file(tmp_file);
        // 余分なプロパティsをx,y,zの前に置く
        file << "ply\n";
        file << "format ascii 1.0\n";
        file << "element vertex 2\n";
        file << "property float s\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";
        file << "end_header\n";
        // s x y z
        file << "9.0  0.0  1.0  2.0\n";
        file << "8.0  3.0  4.0  5.0\n";
        file.close();
    }
    void TearDown() override {
        std::error_code ec; std::filesystem::remove(tmp_file, ec);
    }
};

TEST_F(PlyPropertyOrderTest, ReadAsciiWithExtraProperty) {
    PointCloud cloud;
    ASSERT_TRUE(PlyIO::readPlyFile(tmp_file, cloud));
    ASSERT_EQ(cloud.size(), 2);
    EXPECT_FLOAT_EQ(cloud.points[0].x, 0.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].z, 2.0f);
    EXPECT_FLOAT_EQ(cloud.points[1].x, 3.0f);
    EXPECT_FLOAT_EQ(cloud.points[1].y, 4.0f);
    EXPECT_FLOAT_EQ(cloud.points[1].z, 5.0f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

