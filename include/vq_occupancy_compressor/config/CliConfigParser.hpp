// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_CONFIG_CLI_CONFIG_PARSER_HPP
#define VQ_OCCUPANCY_COMPRESSOR_CONFIG_CLI_CONFIG_PARSER_HPP

#include <string>
#include <vector>

namespace vq_occupancy_compressor::config {

std::string parseConfigPath(const std::vector<std::string>& args);

}  // namespace vq_occupancy_compressor::config

#endif  // VQ_OCCUPANCY_COMPRESSOR_CONFIG_CLI_CONFIG_PARSER_HPP
