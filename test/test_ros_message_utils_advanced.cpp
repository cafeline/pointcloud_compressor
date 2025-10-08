#include <gtest/gtest.h>
#include <vector>
#include <cstdint>
#include "pointcloud_compressor/ros/MessageUtils.hpp"
#include "pointcloud_compressor/msg/pattern_dictionary.hpp"

using namespace pointcloud_compressor;
using namespace pointcloud_compressor::ros;

// 32/64-bitエンコードの往復を検証
TEST(MessageUtilsAdvancedTest, Pack32AndExtract32) {
    // 16bitを超え32bitに収まる値
    std::vector<uint64_t> indices = {0u, 1000u, 65535u, 70000u, 123456789u};

    uint8_t bit_size = 0;
    std::vector<uint8_t> packed;
    MessageUtils::packBlockIndices(indices, bit_size, packed);

    EXPECT_EQ(bit_size, 32);
    // 32bit: 4バイト/要素
    EXPECT_EQ(packed.size(), indices.size() * 4);

    pointcloud_compressor::msg::PatternDictionary msg;
    msg.index_bit_size = bit_size;
    msg.block_indices_data = packed;

    auto extracted = MessageUtils::extractBlockIndices64(msg);
    EXPECT_EQ(extracted.size(), indices.size());
    EXPECT_EQ(extracted, indices);
}

TEST(MessageUtilsAdvancedTest, Pack64AndExtract64) {
    // 32bitを超える値を含む
    std::vector<uint64_t> indices = {0ull, 0xffffffffull, 0x1'0000'0000ull, 0x1234'5678'9abcull};

    uint8_t bit_size = 0;
    std::vector<uint8_t> packed;
    MessageUtils::packBlockIndices(indices, bit_size, packed);

    EXPECT_EQ(bit_size, 64);
    // 64bit: 8バイト/要素
    EXPECT_EQ(packed.size(), indices.size() * 8);

    pointcloud_compressor::msg::PatternDictionary msg;
    msg.index_bit_size = bit_size;
    msg.block_indices_data = packed;

    auto extracted = MessageUtils::extractBlockIndices64(msg);
    EXPECT_EQ(extracted.size(), indices.size());
    EXPECT_EQ(extracted, indices);
}

TEST(MessageUtilsAdvancedTest, MixedForcesWiderBitSize) {
    // 小さい値中心＋1つ大きい値 -> 32bit採用
    std::vector<uint64_t> indices32;
    for (int i = 0; i < 100; ++i) indices32.push_back(i % 200);
    indices32.push_back(100000u);

    uint8_t bit_size = 0;
    std::vector<uint8_t> packed;
    MessageUtils::packBlockIndices(indices32, bit_size, packed);
    EXPECT_EQ(bit_size, 32);

    pointcloud_compressor::msg::PatternDictionary msg32;
    msg32.index_bit_size = bit_size;
    msg32.block_indices_data = packed;
    auto extracted32 = MessageUtils::extractBlockIndices64(msg32);
    EXPECT_EQ(extracted32, indices32);

    // 64bit強制
    std::vector<uint64_t> indices64 = {1ull, 2ull, 3ull, 0x1'0000'0000ull};
    bit_size = 0;
    packed.clear();
    MessageUtils::packBlockIndices(indices64, bit_size, packed);
    EXPECT_EQ(bit_size, 64);

    pointcloud_compressor::msg::PatternDictionary msg64;
    msg64.index_bit_size = bit_size;
    msg64.block_indices_data = packed;
    auto extracted64 = MessageUtils::extractBlockIndices64(msg64);
    EXPECT_EQ(extracted64, indices64);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
