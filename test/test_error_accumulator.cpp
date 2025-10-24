// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "pointcloud_compressor/utils/ErrorAccumulator.hpp"

using pointcloud_compressor::utils::ErrorAccumulator;

TEST(ErrorAccumulatorTest, StartsEmpty) {
    ErrorAccumulator acc;
    EXPECT_TRUE(acc.empty());
    EXPECT_TRUE(acc.str().empty());
}

TEST(ErrorAccumulatorTest, AddsMessagesWithDelimiter) {
    ErrorAccumulator acc;
    acc.add("first");
    acc.add("");
    acc.add("second");

    EXPECT_FALSE(acc.empty());
    EXPECT_EQ("first; second", acc.str());
}

TEST(ErrorAccumulatorTest, ClearsMessages) {
    ErrorAccumulator acc;
    acc.add("error");
    acc.clear();
    EXPECT_TRUE(acc.empty());
    EXPECT_TRUE(acc.str().empty());
}
