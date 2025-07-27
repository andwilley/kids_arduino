#include "gtest/gtest.h"
#include "turret.h"

// A dummy fixture for demonstration purposes.
class TurretTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Set up your test fixture here.
  }

  void TearDown() override {
    // Tear down your test fixture here.
  }

  int meaningOfLife() {
    return 42;
  }
};

TEST_F(TurretTest, MeaningOfLife) {
  EXPECT_EQ(meaningOfLife(), 42);
}
