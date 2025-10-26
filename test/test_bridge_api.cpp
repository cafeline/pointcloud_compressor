#include <gtest/gtest.h>

#include "pointcloud_compressor/bridge/Bridge.hpp"

TEST(BridgeHeaders, CreateAndDestroyRuntimeHandle) {
  PCCRuntimeHandle* handle = pcc_runtime_create();
  ASSERT_NE(handle, nullptr);
  pcc_runtime_destroy(handle);
}

