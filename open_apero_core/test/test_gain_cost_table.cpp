/**
 * @file test_gain_cost_table.cpp
 * @brief GainCostTable's concave-envelope flag: off by default, on when requested at construction.
 */

#include <gtest/gtest.h>

#include "open_apero_core/gain/gain_cost_table.h"

namespace open_apero {
namespace {

Path makePath(double cost, double gain) {
  return Path(gain / cost, gain, cost);
}

TEST(GainCostTableTest, ConcavityCanBeEnabledAtConstruction) {
  GainCostTable table(true);

  table.insert(makePath(1.0, 1.0));
  table.insert(makePath(2.0, 3.0));

  ASSERT_EQ(table.points().size(), 1u);
  EXPECT_DOUBLE_EQ(table.points().front().cost, 2.0);
  EXPECT_DOUBLE_EQ(table.points().front().gain, 3.0);
}

TEST(GainCostTableTest, ConcavityRemainsDisabledByDefault) {
  GainCostTable table;

  table.insert(makePath(1.0, 1.0));
  table.insert(makePath(2.0, 3.0));

  ASSERT_EQ(table.points().size(), 2u);
}

}  // namespace
}  // namespace open_apero
