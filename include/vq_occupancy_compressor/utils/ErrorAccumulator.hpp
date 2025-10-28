// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef VQ_OCCUPANCY_COMPRESSOR_UTILS_ERROR_ACCUMULATOR_HPP
#define VQ_OCCUPANCY_COMPRESSOR_UTILS_ERROR_ACCUMULATOR_HPP

#include <string>

namespace vq_occupancy_compressor::utils {

class ErrorAccumulator {
public:
    void add(const std::string& message) {
        if (message.empty()) {
            return;
        }
        if (!messages_.empty()) {
            messages_ += "; ";
        }
        messages_ += message;
    }

    bool empty() const {
        return messages_.empty();
    }

    const std::string& str() const {
        return messages_;
    }

    void clear() {
        messages_.clear();
    }

private:
    std::string messages_;
};

}  // namespace vq_occupancy_compressor::utils

#endif  // VQ_OCCUPANCY_COMPRESSOR_UTILS_ERROR_ACCUMULATOR_HPP
