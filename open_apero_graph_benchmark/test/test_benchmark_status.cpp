#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_graph_benchmark/benchmark_status.h"

namespace open_apero {
namespace {

std::vector<std::string> statusLines(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream stream(text);
  for (std::string line; std::getline(stream, line);) {
    lines.push_back(line);
  }
  return lines;
}

void expectBoundedFiveLineStatus(const BenchmarkStatus& status) {
  const std::vector<std::string> lines = statusLines(formatBenchmarkStatus(status));
  ASSERT_EQ(lines.size(), kBenchmarkStatusLineCount);
  for (const std::string& line : lines) {
    EXPECT_LE(line.size(), kBenchmarkStatusMaxColumns) << line;
  }
}

BenchmarkStatus representativeStatus() {
  BenchmarkStatus status;
  status.graph_instance = "graph_0";
  status.planner = "dbs_10000";
  status.objective = "expected_gain";
  status.replan_type = "everystep";
  status.phase = "running";
  status.step = 12;
  status.node_id = 742;
  status.step_gain = 83.4;
  status.total_gain = 146.7;
  status.total_cost = 75.4;
  status.cost_budget = 100.0;
  return status;
}

TEST(BenchmarkStatus, FormatsRepresentativeProgressInFiveStableLines) {
  const std::string text = formatBenchmarkStatus(representativeStatus());
  EXPECT_EQ(text,
            "Test  dbs_10000    on graph_0      \n"
            "Using expected_gain,replan is everystep\n"
            "step =    12,  # node =   742,  running\n"
            "gain  +83.4 ->    146.7\n"
            "cost   75.4/   100.0");
  expectBoundedFiveLineStatus(representativeStatus());
}

TEST(BenchmarkStatus, KeepsEveryPhaseAtTheSameLineCount) {
  BenchmarkStatus status = representativeStatus();
  for (const std::string& phase : {"initializing", "running", "finished"}) {
    status.phase = phase;
    EXPECT_NE(formatBenchmarkStatus(status).find(phase), std::string::npos) << phase;
    expectBoundedFiveLineStatus(status);
  }
}

TEST(BenchmarkStatus, BoundsLongLabelsAndLargeValues) {
  BenchmarkStatus status = representativeStatus();
  status.graph_instance = std::string(100, 'g');
  status.planner = std::string(100, 'p');
  status.objective = std::string(100, 'o');
  status.replan_type = std::string(100, 'r');
  status.phase = std::string(100, 's');
  status.step = std::numeric_limits<std::size_t>::max();
  status.node_id = std::numeric_limits<int>::min();
  status.step_gain = -std::numeric_limits<double>::max();
  status.total_gain = std::numeric_limits<double>::max();
  status.total_cost = -std::numeric_limits<double>::max();
  status.cost_budget = std::numeric_limits<double>::max();

  const std::string text = formatBenchmarkStatus(status);
  EXPECT_NE(text.find("..."), std::string::npos);
  EXPECT_EQ(text.find("##"), std::string::npos);
  expectBoundedFiveLineStatus(status);
}

TEST(BenchmarkStatus, HandlesNonFiniteMetricsWithoutChangingShape) {
  BenchmarkStatus status = representativeStatus();
  status.step_gain = std::numeric_limits<double>::quiet_NaN();
  status.total_gain = std::numeric_limits<double>::infinity();
  status.total_cost = -std::numeric_limits<double>::infinity();
  status.cost_budget = std::numeric_limits<double>::quiet_NaN();

  const std::string text = formatBenchmarkStatus(status);
  EXPECT_NE(text.find("nan"), std::string::npos);
  EXPECT_NE(text.find("inf"), std::string::npos);
  expectBoundedFiveLineStatus(status);
}

}  // namespace
}  // namespace open_apero
