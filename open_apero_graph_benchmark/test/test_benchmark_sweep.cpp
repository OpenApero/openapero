#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_graph_benchmark/benchmark_sweep.h"

namespace open_apero {
namespace {

// The launch defaults, pinned: these two sequences decide which result files a default sweep writes.
TEST(BenchmarkSweep, ReproducesTheLaunchDefaultBeamWidths) {
  EXPECT_EQ(geometricBeamWidths(1, 10000, 100), std::vector<int>({1, 100, 10000}));
  EXPECT_EQ(geometricBeamWidths(1, 100, 10), std::vector<int>({1, 10, 100}));
}

// The node's own nh.param fallbacks, which sweep far wider than the launch file does: 26 DBS widths and 21 NBS
// widths, so 69 runs per block rather than 12.
TEST(BenchmarkSweep, ReproducesTheNodeParameterDefaultBeamWidths) {
  const std::vector<int> dbs = geometricBeamWidths(1, 50000000, 2);
  EXPECT_EQ(dbs.size(), 26u);
  EXPECT_EQ(dbs.back(), 33554432);

  const std::vector<int> nbs = geometricBeamWidths(1, 2000000, 2);
  EXPECT_EQ(nbs.size(), 21u);
  EXPECT_EQ(nbs.back(), 1048576);
}

// The bound the guard actually enforces is on the *multiply*, not on the last element: the final width is by
// design the largest one still within max_width, so it may well exceed max_width / step. What must hold is
// that no width wrapped -- doubling 2^30 would be undefined behavior, not a large number.
TEST(BenchmarkSweep, StopsBeforeSignedOverflow) {
  const std::vector<int> widths = geometricBeamWidths(1, std::numeric_limits<int>::max(), 2);
  ASSERT_FALSE(widths.empty());
  EXPECT_EQ(std::adjacent_find(widths.begin(), widths.end(), std::greater_equal<int>()), widths.end())
      << "widths stopped strictly increasing, which is what a wrap looks like";
  for (const int width : widths) {
    EXPECT_GT(width, 0) << "a width wrapped, so the guard let the multiply overflow";
  }
  EXPECT_EQ(widths.size(), 31u);
  EXPECT_EQ(widths.back(), 1073741824) << "2^30 is the largest power of two an int holds";
}

TEST(BenchmarkSweep, HandlesSweepsThatVisitAtMostOneWidth) {
  EXPECT_EQ(geometricBeamWidths(10, 5, 2), std::vector<int>()) << "max below min is an empty sweep, not one width";
  EXPECT_EQ(geometricBeamWidths(7, 7, 2), std::vector<int>({7}));
  EXPECT_EQ(geometricBeamWidths(1, 10, 1000), std::vector<int>({1}));
}

// main()'s validate_beam_sweep rejects all of these before any run starts, so they are unreachable in the
// node. The helper still has to be total: a step of 0 would divide by zero and a step of 1 would never
// terminate, which is a hang rather than a wrong answer.
TEST(BenchmarkSweep, DegenerateInputsAreEmptyRatherThanHanging) {
  EXPECT_EQ(geometricBeamWidths(1, 10, 1), std::vector<int>());
  EXPECT_EQ(geometricBeamWidths(1, 10, 0), std::vector<int>());
  EXPECT_EQ(geometricBeamWidths(1, 10, -2), std::vector<int>());
  EXPECT_EQ(geometricBeamWidths(0, 10, 2), std::vector<int>());
  EXPECT_EQ(geometricBeamWidths(-1, 10, 2), std::vector<int>());
}

TEST(BenchmarkResultFilename, AcceptsWhatTheSweepWritesAndWhatThePlotScriptsGroup) {
  EXPECT_TRUE(isBenchmarkResultFilename("dbs_1.txt"));
  EXPECT_TRUE(isBenchmarkResultFilename("nbs_10000.txt"));
  EXPECT_TRUE(isBenchmarkResultFilename("spt_0.50.txt"));
  EXPECT_TRUE(isBenchmarkResultFilename("tsp_1.00.txt"));
  // GROUPING_PATTERN in python/open_apero_graph_benchmark/benchmark_ranges.py uses re.match, so it groups this too.
  EXPECT_TRUE(isBenchmarkResultFilename("dbs_1_copy.txt"));
}

TEST(BenchmarkResultFilename, RejectsOtherMethodNamesAndNonResults) {
  // Other method names such as dbsx_1.txt are not this benchmark's output, and the Python pattern does not claim
  // them either.
  EXPECT_FALSE(isBenchmarkResultFilename("dbsx_1.txt"));
  EXPECT_FALSE(isBenchmarkResultFilename("nbsx_100.txt"));

  EXPECT_FALSE(isBenchmarkResultFilename("graph.txt"));
  EXPECT_FALSE(isBenchmarkResultFilename("gain_model.txt"));
  EXPECT_FALSE(isBenchmarkResultFilename("dbs.txt"));
  EXPECT_FALSE(isBenchmarkResultFilename("dbs_.txt"));
  EXPECT_FALSE(isBenchmarkResultFilename("dbs_x.txt"));
  EXPECT_FALSE(isBenchmarkResultFilename("dbs_1.csv"));
  EXPECT_FALSE(isBenchmarkResultFilename("dbs_1"));
  EXPECT_FALSE(isBenchmarkResultFilename("prefix_dbs_1.txt"));
}

}  // namespace
}  // namespace open_apero
