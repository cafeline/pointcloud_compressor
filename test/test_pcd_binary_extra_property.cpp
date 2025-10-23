// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "pointcloud_compressor/io/PcdIO.hpp"

using namespace pointcloud_compressor;

class PcdBinaryExtraPropertyTest : public ::testing::Test {
protected:
    std::string tmp_file;
    void SetUp() override {
        tmp_file = std::filesystem::temp_directory_path() / "pcd_binary_extra_property.pcd";
        std::ofstream f(tmp_file, std::ios::binary);
        // ヘッダ: x,y,z,intensity (float)
        f << "# .PCD v0.7 - Point Cloud Data file format\n";
        f << "VERSION 0.7\n";
        f << "FIELDS x y z intensity\n";
        f << "SIZE 4 4 4 4\n";
        f << "TYPE F F F F\n";
        f << "COUNT 1 1 1 1\n";
        f << "WIDTH 2\n";
        f << "HEIGHT 1\n";
        f << "VIEWPOINT 0 0 0 1 0 0 0\n";
        f << "POINTS 2\n";
        f << "DATA binary\n";
        // 2点: (0,1,2,10), (3,4,5,20)
        float data[8] = {0.f,1.f,2.f,10.f, 3.f,4.f,5.f,20.f};
        f.write(reinterpret_cast<const char*>(data), sizeof(data));
        f.close();
    }
    void TearDown() override {
        std::error_code ec; std::filesystem::remove(tmp_file, ec);
    }
};

TEST_F(PcdBinaryExtraPropertyTest, ReadBinaryWithExtraProperty) {
    PointCloud cloud;
    ASSERT_TRUE(PcdIO::readPcdFile(tmp_file, cloud));
    ASSERT_EQ(cloud.size(), 2);
    EXPECT_FLOAT_EQ(cloud.points[0].x, 0.f);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 1.f);
    EXPECT_FLOAT_EQ(cloud.points[0].z, 2.f);
    EXPECT_FLOAT_EQ(cloud.points[1].x, 3.f);
    EXPECT_FLOAT_EQ(cloud.points[1].y, 4.f);
    EXPECT_FLOAT_EQ(cloud.points[1].z, 5.f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

