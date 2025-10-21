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
#include <hdf5_hl.h>

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
        data.dictionary_size = 6;
        data.pattern_bits = 512;
        data.block_size = 8;
        data.grid_origin = {0.0f, 0.0f, 0.0f};

        // Dictionary patterns（各パターン8x8x8ビット=64バイト）
        data.pattern_length = 512;
        data.dictionary_patterns.resize(static_cast<size_t>(data.dictionary_size) * 64);
        for (size_t i = 0; i < data.dictionary_patterns.size(); ++i) {
            data.dictionary_patterns[i] = static_cast<uint8_t>((i * 17) % 256);
        }

        // Block grid information
        data.block_dims = {3, 2, 2};
        data.block_index_bit_width = 3;  // enough for values up to 7
        data.block_index_sentinel = 0;
        const size_t total_cells = static_cast<size_t>(data.block_dims[0]) *
                                   static_cast<size_t>(data.block_dims[1]) *
                                   static_cast<size_t>(data.block_dims[2]);
        data.block_indices.assign(total_cells, 0);

        auto linear_index = [dims = data.block_dims](int bx, int by, int bz) {
            return static_cast<size_t>(bx) +
                   static_cast<size_t>(dims[0]) * (static_cast<size_t>(by) +
                   static_cast<size_t>(dims[1]) * static_cast<size_t>(bz));
        };

        data.grid_dimensions = {
            data.block_dims[0] * static_cast<int32_t>(data.block_size),
            data.block_dims[1] * static_cast<int32_t>(data.block_size),
            data.block_dims[2] * static_cast<int32_t>(data.block_size)};

        data.block_indices[linear_index(0, 0, 0)] = 0;
        data.block_indices[linear_index(1, 0, 0)] = 5;
        data.block_indices[linear_index(2, 1, 0)] = 3;
        data.block_indices[linear_index(1, 1, 1)] = 2;

        data.original_points = 10000;
        data.compressed_voxels = 4;
        data.compression_ratio = static_cast<double>(data.original_points) /
                                static_cast<double>(data.compressed_voxels);
        data.bounding_box_min = {0.0, 0.0, 0.0};
        data.bounding_box_max = {
            static_cast<double>(data.grid_dimensions[0]) * data.voxel_size,
            static_cast<double>(data.grid_dimensions[1]) * data.voxel_size,
            static_cast<double>(data.grid_dimensions[2]) * data.voxel_size};

        return data;
    }

    std::vector<uint32_t> readBlockIndices() {
        std::vector<uint32_t> values;
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

            hid_t dtype = H5Dget_type(dataset);
            const size_t type_size = H5Tget_size(dtype);
            std::vector<uint8_t> packed(static_cast<size_t>(dims));
            if (type_size == sizeof(uint8_t)) {
                H5Dread(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, packed.data());
            } else {
                ADD_FAILURE() << "Unexpected block_indices dataset type size: " << type_size;
            }
            H5Tclose(dtype);
            H5Sclose(dataspace);
            H5Dclose(dataset);

            const auto dims_trip = readInt32Triple("/compressed_data/block_dims");
            const size_t total_blocks = static_cast<size_t>(dims_trip[0]) *
                                        static_cast<size_t>(dims_trip[1]) *
                                        static_cast<size_t>(dims_trip[2]);
            const uint32_t bit_width = readUint32Scalar("/compression_params/block_index_bit_width");
            if (bit_width == 0U || bit_width > 32U) {
                ADD_FAILURE() << "Unexpected block_index_bit_width=" << bit_width;
                H5Fclose(file_id);
                return values;
            }
            values.assign(total_blocks, 0U);
            for (size_t i = 0; i < total_blocks; ++i) {
                uint32_t value = 0;
                for (uint32_t bit = 0; bit < bit_width; ++bit) {
                    const size_t absolute_bit = i * static_cast<size_t>(bit_width) + bit;
                    const size_t byte_index = absolute_bit >> 3U;
                    const size_t bit_index = absolute_bit & 7U;
                    if (byte_index < packed.size()) {
                        const uint8_t byte = packed[byte_index];
                        const uint8_t bit_value = static_cast<uint8_t>((byte >> bit_index) & 0x1U);
                        value |= static_cast<uint32_t>(bit_value) << bit;
                    }
                }
                values[i] = value;
            }
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
};

TEST_F(HDF5IOTest, WriteCreatesBlockIndexDatasets) {
    HDF5IO io;
    ASSERT_TRUE(io.write(test_file_, test_data_)) << io.getLastError();
    ASSERT_TRUE(std::filesystem::exists(test_file_));

    auto indices = readBlockIndices();
    ASSERT_EQ(indices.size(), test_data_.block_indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        EXPECT_EQ(indices[i], test_data_.block_indices[i]) << "Mismatch at index " << i;
    }

    auto dims = readInt32Triple("/compressed_data/block_dims");
    EXPECT_EQ(dims, test_data_.block_dims);

    auto bit_width = readUint32Scalar("/compression_params/block_index_bit_width");
    EXPECT_EQ(bit_width, static_cast<uint32_t>(test_data_.block_index_bit_width));

}

TEST_F(HDF5IOTest, RawVoxelGridStoresVoxelsWithFreeSpace) {
    HDF5IO io;
    HDF5IO::RawVoxelGridData raw;

    raw.dim_x = 2;
    raw.dim_y = 2;
    raw.dim_z = 1;
    raw.voxel_size = 0.25f;
    raw.origin = {1.0f, -2.0f, 0.5f};
    raw.voxel_values = {0U, 255U, 0U, 255U};
    raw.occupied_voxels = {
        {1, 0, 0},
        {1, 1, 0},
    };

    ASSERT_TRUE(io.writeRawVoxelGrid(test_file_, raw)) << io.getLastError();

    hid_t file_id = H5Fopen(test_file_.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    ASSERT_GE(file_id, 0);

    hid_t dataset = H5Dopen2(file_id, "/raw_voxel_grid/voxel_values", H5P_DEFAULT);
    ASSERT_GE(dataset, 0);

    hid_t dataspace = H5Dget_space(dataset);
    ASSERT_GE(dataspace, 0);

    const int ndims = H5Sget_simple_extent_ndims(dataspace);
    EXPECT_EQ(ndims, 3);

    hsize_t dims[3] = {0, 0, 0};
    H5Sget_simple_extent_dims(dataspace, dims, nullptr);
    EXPECT_EQ(dims[0], static_cast<hsize_t>(raw.dim_z));
    EXPECT_EQ(dims[1], static_cast<hsize_t>(raw.dim_y));
    EXPECT_EQ(dims[2], static_cast<hsize_t>(raw.dim_x));

    std::vector<uint8_t> values(static_cast<size_t>(raw.dim_x) *
                                static_cast<size_t>(raw.dim_y) *
                                static_cast<size_t>(raw.dim_z),
                                0U);
    H5Dread(dataset, H5T_NATIVE_UINT8, H5S_ALL, H5S_ALL, H5P_DEFAULT, values.data());
    EXPECT_EQ(values, raw.voxel_values);

    H5Sclose(dataspace);
    H5Dclose(dataset);
    H5Fclose(file_id);
}

TEST_F(HDF5IOTest, LegacyDatasetsAreNotWritten) {
    HDF5IO io;
    ASSERT_TRUE(io.write(test_file_, test_data_)) << io.getLastError();

    EXPECT_FALSE(legacyDatasetExists("indices"));
    EXPECT_FALSE(legacyDatasetExists("voxel_positions"));
}

TEST_F(HDF5IOTest, ReadFailsWithoutBlockIndices) {
    const std::string legacy_file = test_file_ + "_legacy";
    hid_t file = H5Fcreate(legacy_file.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    ASSERT_GE(file, 0);

    hid_t params = H5Gcreate2(file, "/compression_params", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    ASSERT_GE(params, 0);
    uint32_t block_size = 8;
    float voxel_size = 0.5f;
    float origin[3] = {0.0f, 0.0f, 0.0f};
    const hsize_t scalar_dim[1] = {1};
    const hsize_t origin_dim[1] = {3};
    H5LTmake_dataset(params, "block_size", 1, scalar_dim, H5T_NATIVE_UINT32, &block_size);
    H5LTmake_dataset(params, "voxel_size", 1, scalar_dim, H5T_NATIVE_FLOAT, &voxel_size);
    H5LTmake_dataset(params, "grid_origin", 1, origin_dim, H5T_NATIVE_FLOAT, origin);
    H5Gclose(params);

    hid_t dict = H5Gcreate2(file, "/dictionary", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    ASSERT_GE(dict, 0);
    uint32_t pattern_length = 512;
    std::vector<uint8_t> pattern(64, 0xFF);
    H5LTmake_dataset(dict, "pattern_length", 1, scalar_dim, H5T_NATIVE_UINT32, &pattern_length);
    const hsize_t pattern_dims[1] = {static_cast<hsize_t>(pattern.size())};
    H5LTmake_dataset(dict, "patterns", 1, pattern_dims, H5T_NATIVE_UINT8, pattern.data());
    H5Gclose(dict);

    hid_t comp = H5Gcreate2(file, "/compressed_data", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    ASSERT_GE(comp, 0);
    uint16_t idx = 0;
    int32_t pos[3] = {0, 0, 0};
    H5LTmake_dataset(comp, "indices", 1, scalar_dim, H5T_NATIVE_UINT16, &idx);
    const hsize_t voxel_pos_dims[2] = {1, 3};
    H5LTmake_dataset(comp, "voxel_positions", 2, voxel_pos_dims, H5T_NATIVE_INT32, pos);
    H5Gclose(comp);

    H5Fclose(file);

    HDF5IO io;
    CompressedMapData data;
    EXPECT_FALSE(io.read(legacy_file, data));
    EXPECT_FALSE(io.getLastError().empty());

    std::filesystem::remove(legacy_file);
}

TEST_F(HDF5IOTest, IsValidHDF5) {
    HDF5IO io;

    EXPECT_FALSE(io.isValidHDF5("/tmp/non_existent_file.h5"));

    ASSERT_TRUE(io.write(test_file_, test_data_)) << io.getLastError();
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
    large_data.block_index_bit_width = 5;
    large_data.block_index_sentinel = 0;
    large_data.grid_dimensions = {
        large_data.block_dims[0] * static_cast<int32_t>(large_data.block_size),
        large_data.block_dims[1] * static_cast<int32_t>(large_data.block_size),
        large_data.block_dims[2] * static_cast<int32_t>(large_data.block_size)};
    large_data.bounding_box_max = {
        static_cast<double>(large_data.grid_dimensions[0]) * large_data.voxel_size,
        static_cast<double>(large_data.grid_dimensions[1]) * large_data.voxel_size,
        static_cast<double>(large_data.grid_dimensions[2]) * large_data.voxel_size};
    large_data.block_indices.assign(num_cells, 0);
    for (size_t i = 0; i < num_cells; ++i) {
        large_data.block_indices[i] = static_cast<uint32_t>((i * 7) % 23);
    }
    large_data.compressed_voxels = num_cells;

    ASSERT_TRUE(io.write(test_file_, large_data));

    auto indices = readBlockIndices();
    ASSERT_EQ(indices.size(), num_cells);
    EXPECT_EQ(indices.front(), static_cast<uint32_t>(0));
    const uint32_t expected_last = static_cast<uint32_t>(((num_cells - 1) * 7) % 23);
    EXPECT_EQ(indices.back(), expected_last);
}

TEST_F(HDF5IOTest, MetadataPreservation) {
    HDF5IO io;

    test_data_.frame_id = "custom_frame";
    test_data_.compression_method = "advanced_voxel";
    test_data_.version = "2.0.0";

    ASSERT_TRUE(io.write(test_file_, test_data_)) << io.getLastError();

    CompressedMapData read_data;
    ASSERT_TRUE(io.read(test_file_, read_data)) << io.getLastError();

    EXPECT_EQ(read_data.frame_id, "custom_frame");
    EXPECT_EQ(read_data.compression_method, "advanced_voxel");
    EXPECT_EQ(read_data.version, "2.0.0");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
