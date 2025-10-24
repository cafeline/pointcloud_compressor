// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_COMPRESSOR_CONFIG_CLI_CONFIG_PARSER_HPP
#define POINTCLOUD_COMPRESSOR_CONFIG_CLI_CONFIG_PARSER_HPP

#include <string>
#include <vector>

namespace pointcloud_compressor::config {

std::string parseConfigPath(const std::vector<std::string>& args);

}  // namespace pointcloud_compressor::config

#endif  // POINTCLOUD_COMPRESSOR_CONFIG_CLI_CONFIG_PARSER_HPP
