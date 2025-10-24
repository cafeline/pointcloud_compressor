// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "pointcloud_compressor/config/CliConfigParser.hpp"

using pointcloud_compressor::config::parseConfigPath;

TEST(CliConfigParserTest, ExtractsPathFromSingleArgument) {
    std::vector<std::string> args = {"config.yaml"};
    EXPECT_EQ("config.yaml", parseConfigPath(args));
}

TEST(CliConfigParserTest, RejectsMissingArgument) {
    std::vector<std::string> args;
    EXPECT_THROW(parseConfigPath(args), std::invalid_argument);
}

TEST(CliConfigParserTest, RejectsExtraArguments) {
    std::vector<std::string> args = {"one.yaml", "two.yaml"};
    EXPECT_THROW(parseConfigPath(args), std::invalid_argument);
}
