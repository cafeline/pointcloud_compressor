#include <gtest/gtest.h>

#include "vq_occupancy_compressor/bridge/Bridge.hpp"

TEST(BridgeHeaders, CreateAndDestroyCompressionHandle) {
  VqoCompressionHandle* handle = vqo_handle_create();
  ASSERT_NE(handle, nullptr);
  vqo_handle_destroy(handle);
}
