// Smoke test for the data handler module. The full read/convert pipeline is
// Qt + file-I/O driven and belongs to integration tests with data fixtures;
// here we just verify the module links and starts in a clean state. The
// conversion logic it relies on is covered by the Core converter/KeyFrame tests.
#include <gtest/gtest.h>

#include <QFileInfoList>

#include "Handler/ImageDataHandler.h"

using namespace MapCreator;

TEST(ImageHandler2, StartsWithNoKeyFrames) {
  ImageHandler2 handler{QFileInfoList{}};
  EXPECT_TRUE(handler.GetKeyFrames().empty());
}
