#include <gtest/gtest.h>

#include "pointcloud_compressor/bridge/Bridge.hpp"

TEST(BridgeHeaders, CreateAndDestroyCompressionHandle) {
  PCCCompressionHandle* handle = pcc_handle_create();
  ASSERT_NE(handle, nullptr);
  pcc_handle_destroy(handle);
}
