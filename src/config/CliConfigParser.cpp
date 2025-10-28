// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "vq_occupancy_compressor/config/CliConfigParser.hpp"

#include <stdexcept>

namespace vq_occupancy_compressor::config {

std::string parseConfigPath(const std::vector<std::string>& args) {
    if (args.size() != 1) {
        throw std::invalid_argument("Expected exactly one configuration path argument");
    }
    if (args.front().empty()) {
        throw std::invalid_argument("Configuration path must not be empty");
    }
    return args.front();
}

}  
