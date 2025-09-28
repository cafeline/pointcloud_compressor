#include <gtest/gtest.h>

#include <pointcloud_compressor/io/HDF5IO.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include <cstdint>

#include <hdf5.h>

using namespace pointcloud_compressor;

class HDF5IOTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_data_ = createTestData();
        test_file_ = "/tmp/test_compressed_map_" +
                     std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
                     ".h5";
    }

    void TearDown() override {
        if (std::filesystem::exists(test_file_)) {
            std::filesystem::remove(test_file_);
        }
    }

    CompressedMapData createTestData() {
        CompressedMapData data;

        // Metadata
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
        data.creation_time = ss.str();

        // Compression parameters
        data.voxel_size = 0.1f;
        data.dictionary_size = 256;
        data.pattern_bits = 8;
        data.block_size = 8;

        // Dictionary patterns (8x8x8 bits -> 64 bytes per pattern)
        data.pattern_length = 512;
        data.dictionary_patterns.resize(data.dictionary_size * 64);
        for (size_t i = 0; i < data.dictionary_patterns.size(); ++i) {
            data.dictionary_patterns[i] = static_cast<uint8_t>(i % 256);
        }

        // Block grid information
        data.block_offset = {-2, 3, 5};
        data.block_dims = {3, 2, 2};
        data.block_index_bit_width = 16;
        data.block_index_sentinel = sentinel_;
        const size_t total_cells = static_cast<size_t>(data.block_dims[0]) *
                                   static_cast<size_t>(data.block_dims[1]) *
                                   static_cast<size_t>(data.block_dims[2]);
        data.block_indices.assign(total_cells, sentinel_);

        auto linear_index = [dims = data.block_dims](int bx, int by, int bz) {
            return static_cast<size_t>(bx) +
                   static_cast<size_t>(dims[0]) * (static_cast<size_t>(by) +
                   static_cast<size_t>(dims[1]) * static_cast<size_t>(bz));
        };

        data.block_indices[linear_index(0, 0, 0)] = 0;
        data.block_indices[linear_index(1, 0, 0)] = 1;
        data.block_indices[linear_index(2, 1, 0)] = 42;
        data.block_indices[linear_index(1, 1, 1)] = 128;

        data.original_points = 10000;
        data.compressed_voxels = 4;
        data.compression_ratio = static_cast<double>(data.original_points) /
                                static_cast<double>(data.compressed_voxels);
        data.bounding_box_min = {-10.0, -10.0, -10.0};
        data.bounding_box_max = {10.0, 10.0, 10.0};

        return data;
    }

    std::vector<uint64_t> readBlockIndices() {
        std::vector<uint64_t> values;
        hid_t file_id = H5Fopen(test_file_.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
        EXPECT_GE(file_id, 0);
        if (file_id < 0) {
            return values;
        }

        hid_t dataset = H5Dopen2(file_id, "/compressed_data/block_indices", H5P_DEFAULT);
        EXPECT_GE(dataset, 0);
        if (dataset >= 0) {
            hid_t dataspace = H5Dget_space(dataset);
            hsize_t dims;
            H5Sget_simple_extent_dims(dataspace, &dims, nullptr);
            values.resize(dims);

            hid_t dtype = H5Dget_type(dataset);
            const size_t type_size = H5Tget_size(dtype);
            if (type_size == sizeof(uint8_t)) {
                std::vector<uint8_t> tmp(dims);
                H5Dread(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, tmp.data());
                std::transform(tmp.begin(), tmp.end(), values.begin(), [](uint8_t v) {
                    return static_cast<uint64_t>(v);
                });
            } else if (type_size == sizeof(uint16_t)) {
                std::vector<uint16_t> tmp(dims);
                H5Dread(dataset, H5T_NATIVE_UINT16, H5S_ALL, H5S_ALL, H5P_DEFAULT, tmp.data());
                std::transform(tmp.begin(), tmp.end(), values.begin(), [](uint16_t v) {
                    return static_cast<uint64_t>(v);
                });
            } else if (type_size == sizeof(uint32_t)) {
                std::vector<uint32_t> tmp(dims);
                H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, tmp.data());
                std::transform(tmp.begin(), tmp.end(), values.begin(), [](uint32_t v) {
                    return static_cast<uint64_t>(v);
                });
            } else {
                ADD_FAILURE() << "Unexpected block_indices dataset type size: " << type_size;
            }
            H5Tclose(dtype);
            H5Sclose(dataspace);
            H5Dclose(dataset);
        }

        H5Fclose(file_id);
        return values;
    }

    std::array<int32_t, 3> readInt32Triple(const std::string& dataset_path) {
        std::array<int32_t, 3> values{0, 0, 0};
        hid_t file_id = H5Fopen(test_file_.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
        EXPECT_GE(file_id, 0);
        if (file_id < 0) {
            return values;
        }

        hid_t dataset = H5Dopen2(file_id, dataset_path.c_str(), H5P_DEFAULT);
        EXPECT_GE(dataset, 0);
        if (dataset >= 0) {
            H5Dread(dataset, H5T_NATIVE_INT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, values.data());
            H5Dclose(dataset);
        }

        H5Fclose(file_id);
        return values;
    }

    uint32_t readUint32Scalar(const std::string& dataset_path) {
        uint32_t value = 0;
        hid_t file_id = H5Fopen(test_file_.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
        EXPECT_GE(file_id, 0);
        if (file_id < 0) {
            return value;
        }

        hid_t dataset = H5Dopen2(file_id, dataset_path.c_str(), H5P_DEFAULT);
        EXPECT_GE(dataset, 0);
        if (dataset >= 0) {
            H5Dread(dataset, H5T_NATIVE_UINT32, H5S_ALL, H5S_ALL, H5P_DEFAULT, &value);
            H5Dclose(dataset);
        }

        H5Fclose(file_id);
        return value;
    }

    bool legacyDatasetExists(const std::string& name) {
        hid_t file_id = H5Fopen(test_file_.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
        if (file_id < 0) {
            return false;
        }

        bool exists = false;
        if (H5Lexists(file_id, "/compressed_data", H5P_DEFAULT) > 0) {
            hid_t group_id = H5Gopen2(file_id, "/compressed_data", H5P_DEFAULT);
            if (group_id >= 0) {
                exists = H5Lexists(group_id, name.c_str(), H5P_DEFAULT) > 0;
                H5Gclose(group_id);
            }
        }

        H5Fclose(file_id);
        return exists;
    }

    CompressedMapData test_data_;
    std::string test_file_;
    const uint64_t sentinel_ = std::numeric_limits<uint16_t>::max();
};

TEST_F(HDF5IOTest, WriteCreatesBlockIndexDatasets) {
    HDF5IO io;
    ASSERT_TRUE(io.write(test_file_, test_data_));
    ASSERT_TRUE(std::filesystem::exists(test_file_));

    auto indices = readBlockIndices();
    ASSERT_EQ(indices.size(), test_data_.block_indices.size());
    EXPECT_EQ(indices, test_data_.block_indices);

    auto offset = readInt32Triple("/compressed_data/block_offset");
    auto dims = readInt32Triple("/compressed_data/block_dims");
    EXPECT_EQ(offset, test_data_.block_offset);
    EXPECT_EQ(dims, test_data_.block_dims);

    auto bit_width = readUint32Scalar("/compression_params/block_index_bit_width");
    EXPECT_EQ(bit_width, static_cast<uint32_t>(test_data_.block_index_bit_width));
}

TEST_F(HDF5IOTest, LegacyDatasetsAreNotWritten) {
    HDF5IO io;
    ASSERT_TRUE(io.write(test_file_, test_data_));

    EXPECT_FALSE(legacyDatasetExists("indices"));
    EXPECT_FALSE(legacyDatasetExists("voxel_positions"));
}

TEST_F(HDF5IOTest, IsValidHDF5) {
    HDF5IO io;

    EXPECT_FALSE(io.isValidHDF5("/tmp/non_existent_file.h5"));

    ASSERT_TRUE(io.write(test_file_, test_data_));
    EXPECT_TRUE(io.isValidHDF5(test_file_));

    std::string invalid_file = "/tmp/invalid_file.h5";
    std::ofstream ofs(invalid_file);
    ofs << "This is not an HDF5 file";
    ofs.close();
    EXPECT_FALSE(io.isValidHDF5(invalid_file));
    std::filesystem::remove(invalid_file);
}

TEST_F(HDF5IOTest, ReadNonExistentFile) {
    HDF5IO io;
    CompressedMapData data;

    EXPECT_FALSE(io.read("/tmp/non_existent_file.h5", data));
    EXPECT_FALSE(io.getLastError().empty());
}

TEST_F(HDF5IOTest, WriteToInvalidPath) {
    HDF5IO io;
    EXPECT_FALSE(io.write("/invalid/path/file.h5", test_data_));
    EXPECT_FALSE(io.getLastError().empty());
}

TEST_F(HDF5IOTest, LargeDataset) {
    HDF5IO io;
    CompressedMapData large_data = test_data_;

    const size_t num_cells = 100000;
    large_data.block_dims = {100, 100, 10};
    large_data.block_offset = {0, 0, 0};
    large_data.block_index_bit_width = 16;
    large_data.block_index_sentinel = sentinel_;
    large_data.block_indices.assign(num_cells, sentinel_);
    for (size_t i = 0; i < num_cells; ++i) {
        large_data.block_indices[i] = static_cast<uint64_t>(i % 256);
    }
    large_data.compressed_voxels = num_cells;

    ASSERT_TRUE(io.write(test_file_, large_data));

    auto indices = readBlockIndices();
    ASSERT_EQ(indices.size(), num_cells);
    EXPECT_EQ(indices.front(), 0u);
    EXPECT_EQ(indices.back(), static_cast<uint64_t>((num_cells - 1) % 256));
}

TEST_F(HDF5IOTest, MetadataPreservation) {
    HDF5IO io;

    test_data_.frame_id = "custom_frame";
    test_data_.compression_method = "advanced_voxel";
    test_data_.version = "2.0.0";

    ASSERT_TRUE(io.write(test_file_, test_data_));

    CompressedMapData read_data;
    ASSERT_TRUE(io.read(test_file_, read_data));

    EXPECT_EQ(read_data.frame_id, "custom_frame");
    EXPECT_EQ(read_data.compression_method, "advanced_voxel");
    EXPECT_EQ(read_data.version, "2.0.0");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
