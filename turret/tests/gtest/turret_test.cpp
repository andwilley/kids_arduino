#include "turret.h"
#include "gtest/gtest.h"
#include <cmath>

// The fixture for testing class Turret.
class TurretTest : public ::testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TurretTest() {
    // You can do set-up work for each test here.
  }

  ~TurretTest() override {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Turret.
};

TEST_F(TurretTest, FindHeatCenter_HotspotOffCenter) {
  float temps[AMG88xx_PIXEL_ARRAY_SIZE];
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; ++i) {
    temps[i] = 20.0; // Below threshold
  }

  // Create a 2x2 hotspot off-center
  temps[18] = 30.0;
  temps[19] = 30.0;
  temps[26] = 30.0;
  temps[27] = 30.0;

  Point<float> center = FindHeatCenter(temps, AMG88xx_PIXEL_ARRAY_SIZE);

  // The hotspot is at indices 18, 19, 26, 27.
  // Coords are (2,2), (3,2), (2,3), (3,3)
  // Center should be (2.5, 2.5)
  // kGridMidPt = (8-1)/2 = 3.5
  // kSensorFov = 31.5
  // kGridToAngle = 31.5 / 3.5 = 9
  // Expected X: (2.5 - 3.5) * 9 = -9
  // Expected Y: (2.5 - 3.5) * 9 = 9
  EXPECT_NEAR(center.x, -9.0, 1e-5);
  EXPECT_NEAR(center.y, 9.0, 1e-5);
}
