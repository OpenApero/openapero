/**
 * @file test_logging.cpp
 * @brief Drives the logging path directly (no sim): CSV header/rows, the
 *        deterministic schema fingerprint (cross-checked against the Python
 *        reference constant), manifest emission, and per-row flush durability
 *        under a hard crash (SIGKILL, which no destructor can survive).
 */

#include <dirent.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "open_apero_core/logging/metrics.h"
#include "open_apero_core/logging/performance_log.h"
#include "open_apero_core/logging/run_manifest.h"
#include "open_apero_core/logging/schema_fingerprint.h"

namespace open_apero {
namespace {

// Cross-language reference: open_apero_analysis/test/test_fingerprint.py pins
// the same schema to this value. If the encoding drifts on either side, both
// this test and the Python test must be updated together — by design.
constexpr const char* kReferenceFingerprint = "8da7ad974ddeb2ba";

struct HarnessMetrics : Metrics {
  double a = 0.0;
  int b = 0;
  void resetCycle() override {}
  void resetEpisode() override {}
  void declareFields(MetricsFields& fields) const override {
    fields.add("A", &a, "s");
    fields.add("B", &b, "count", ParentRef::global("A"));
  }
};

std::string readFile(const std::string& path) {
  std::ifstream file(path);
  std::stringstream ss;
  ss << file.rdbuf();
  return ss.str();
}

std::string tmpPath(const std::string& name) {
  return ::testing::TempDir() + "/open_apero_test_" + name;
}

// The fixed schema shared with the Python fingerprint test.
std::vector<PerformanceLog::Column> referenceColumns() {
  static double d = 0.0;
  static int i = 0;
  return {
      {"TotalCost", &d, "double", "cost", ""},
      {"planning.GenerateNode", &d, "double", "s", "planning.ExpandTime"},
      {"planning.PlanPathTime", &d, "double", "s", "PlanPathTotal"},
      {"NumExpansions", &i, "int", "count", ""},
  };
}

TEST(PerformanceLogTest, WritesHeaderAndRowsFromDeclarations) {
  HarnessMetrics metrics;
  PerformanceLog log;
  log.addMetrics(metrics);

  const std::string path = tmpPath("perf.csv");
  ASSERT_TRUE(log.open(path));
  metrics.a = 1.5;
  metrics.b = 3;
  log.writeRow();
  metrics.a = 2.5;
  metrics.b = 7;
  log.writeRow();
  log.close();

  std::ifstream file(path);
  std::string header, row1, row2;
  std::getline(file, header);
  std::getline(file, row1);
  std::getline(file, row2);
  EXPECT_EQ(header, "A,B");
  EXPECT_EQ(row1, "1.5,3");
  EXPECT_EQ(row2, "2.5,7");
}

TEST(PerformanceLogTest, DoublesKeepFifteenSignificantDigits) {
  HarnessMetrics metrics;
  PerformanceLog log;
  log.addMetrics(metrics);

  const std::string path = tmpPath("precision.csv");
  ASSERT_TRUE(log.open(path));
  metrics.a = 1791234567.123456;  // a ROS time stamp
  metrics.b = 1234567;
  log.writeRow();
  metrics.a = 0.1;
  log.writeRow();
  log.close();

  std::ifstream file(path);
  std::string header, row1, row2;
  std::getline(file, header);
  std::getline(file, row1);
  std::getline(file, row2);
  EXPECT_EQ(row1, "1791234567.12346,1234567");
  EXPECT_EQ(row2, "0.1,1234567");
}

TEST(PerformanceLogTest, AddMetricsAfterOpenThrows) {
  HarnessMetrics metrics;
  PerformanceLog log;
  log.addMetrics(metrics);
  ASSERT_TRUE(log.open(tmpPath("guard.csv")));
  EXPECT_THROW(log.addMetrics(metrics), std::logic_error);
}

TEST(SchemaFingerprintTest, MatchesPythonReferenceConstant) {
  EXPECT_EQ(schemaFingerprint(referenceColumns()), kReferenceFingerprint);
}

TEST(SchemaFingerprintTest, Fnv1aKnownVectors) {
  EXPECT_EQ(fnv1a64(""), kFnvOffsetBasis);
  EXPECT_EQ(fnv1a64("a"), 0xAF63DC4C8601EC8CULL);
  EXPECT_EQ(fnv1a64("foobar"), 0x85944171F73967E8ULL);
}

TEST(RunManifestTest, WritesYamlWithFingerprintProvenanceAndSchema) {
  RunManifest manifest;
  manifest.columns = referenceColumns();
  manifest.layers = {{"planning", "PlanningLayer"}, {"backtracking", "BacktrackingLayer"}};
  manifest.cost_budget = 600.0;
  manifest.random_seed = 42;

  const std::string path = tmpPath("run_manifest.yaml");
  ::unlink(path.c_str());  // write() publishes, it does not truncate.
  ASSERT_TRUE(manifest.write(path));
  const std::string yaml = readFile(path);

  EXPECT_NE(yaml.find(std::string("schema_fingerprint: \"") + kReferenceFingerprint + "\""), std::string::npos);
  EXPECT_NE(yaml.find("build_git_sha:"), std::string::npos);
  EXPECT_NE(yaml.find("build_git_dirty:"), std::string::npos);
  EXPECT_NE(yaml.find("build_components:"), std::string::npos);
  EXPECT_NE(yaml.find("start_utc:"), std::string::npos);
  EXPECT_NE(yaml.find("cost_budget: 600"), std::string::npos);
  EXPECT_NE(yaml.find("\nrandom_seed: 42\n"), std::string::npos);
  EXPECT_NE(yaml.find("planning.GenerateNode"), std::string::npos);
  EXPECT_NE(yaml.find("parent: \"planning.ExpandTime\""), std::string::npos);
}

TEST(RunManifestTest, PublishesWithGroupReadablePermissions) {
  RunManifest manifest;
  manifest.columns = referenceColumns();

  const std::string path = tmpPath("run_manifest_mode.yaml");
  ::unlink(path.c_str());
  ASSERT_TRUE(manifest.write(path));

  struct stat file_status {};
  ASSERT_EQ(::stat(path.c_str(), &file_status), 0);
  EXPECT_EQ(file_status.st_mode & 0777, 0644);
}

// Provenance must name each repository, and every emitted component must be
// complete: an entry with an empty name or SHA reads as "provenance recorded"
// while identifying nothing. Asserted through the YAML rather than build_info.h,
// which stays private to the library that generates it.
TEST(RunManifestTest, NamesEveryBuildComponentWithARevision) {
  RunManifest manifest;
  manifest.columns = referenceColumns();
  const std::string path = tmpPath("run_manifest_components.yaml");
  ::unlink(path.c_str());
  ASSERT_TRUE(manifest.write(path));

  const std::string yaml = readFile(path);
  const size_t begin = yaml.find("build_components:\n");
  ASSERT_NE(begin, std::string::npos);
  const size_t end = yaml.find("start_utc:", begin);
  ASSERT_NE(end, std::string::npos);
  const std::string block = yaml.substr(begin, end - begin);

  const auto quotedAfter = [](const std::string& line, const std::string& key) {
    const size_t at = line.find(key + ": \"");
    if (at == std::string::npos) return std::string();
    const size_t from = at + key.size() + 3;
    return line.substr(from, line.find('"', from) - from);
  };

  std::istringstream lines(block);
  std::string line;
  std::getline(lines, line);  // drop the "build_components:" key itself
  int num_entries = 0;
  std::string first_sha;
  while (std::getline(lines, line)) {
    if (line.find("- {") == std::string::npos) continue;
    ++num_entries;
    EXPECT_FALSE(quotedAfter(line, "name").empty()) << line;
    EXPECT_FALSE(quotedAfter(line, "sha").empty()) << line;
    if (num_entries == 1) first_sha = quotedAfter(line, "sha");
  }
  EXPECT_GE(num_entries, 1);
  // build_git_sha is documented as an alias of the first component.
  EXPECT_NE(yaml.find("build_git_sha: \"" + first_sha + "\""), std::string::npos);
}

TEST(RunManifestTest, RefusesAPreExistingTargetAndLeavesItIntact) {
  RunManifest manifest;
  manifest.columns = referenceColumns();

  const std::string path = tmpPath("run_manifest_existing.yaml");
  ::unlink(path.c_str());
  {
    std::ofstream existing(path);
    existing << "existing contents\n";
  }

  std::string error_message;
  EXPECT_FALSE(manifest.write(path, &error_message));
  EXPECT_NE(error_message.find("link failed"), std::string::npos);
  EXPECT_NE(error_message.find("errno " + std::to_string(EEXIST)), std::string::npos);
  EXPECT_EQ(readFile(path), "existing contents\n");
}

TEST(RunManifestTest, FailsWithoutCreatingAnythingWhenTheDirectoryIsUnwritable) {
  RunManifest manifest;
  manifest.columns = referenceColumns();

  const std::string path = tmpPath("no_such_dir_open_apero/run_manifest.yaml");
  std::string error_message;
  EXPECT_FALSE(manifest.write(path, &error_message));
  EXPECT_NE(error_message.find("mkstemp failed"), std::string::npos);
  EXPECT_NE(error_message.find("errno " + std::to_string(ENOENT)), std::string::npos);
  EXPECT_NE(::access(path.c_str(), F_OK), 0);
}

// The publish step must not leave its scratch file behind, or a run directory
// accumulates run_manifest.yaml.XXXXXX debris that no consumer accounts for.
TEST(RunManifestTest, LeavesNoTemporaryFileBehind) {
  RunManifest manifest;
  manifest.columns = referenceColumns();

  const std::string dir = tmpPath("manifest_tmp_scan");
  ::mkdir(dir.c_str(), 0775);
  const std::string path = dir + "/run_manifest.yaml";
  ::unlink(path.c_str());

  ASSERT_TRUE(manifest.write(path));

  std::vector<std::string> entries;
  DIR* handle = ::opendir(dir.c_str());
  ASSERT_NE(handle, nullptr);
  while (const dirent* entry = ::readdir(handle)) {
    const std::string name = entry->d_name;
    if (name != "." && name != "..") entries.push_back(name);
  }
  ::closedir(handle);

  EXPECT_EQ(entries.size(), 1u);
  EXPECT_EQ(entries.front(), "run_manifest.yaml");
}

// serialize() is what write() publishes; keeping them identical is what makes
// the atomic path trustworthy.
TEST(RunManifestTest, SerializeMatchesThePublishedFile) {
  RunManifest manifest;
  manifest.columns = referenceColumns();
  manifest.cost_budget = 600.0;

  const std::string path = tmpPath("run_manifest_serialize.yaml");
  ::unlink(path.c_str());
  const std::string rendered = manifest.serialize();
  ASSERT_TRUE(manifest.write(path));

  // start_utc is a wall clock, so compare everything up to it and the schema
  // below it rather than demanding byte equality across a second boundary.
  const std::string published = readFile(path);
  const std::string prefix = rendered.substr(0, rendered.find("start_utc:"));
  EXPECT_EQ(published.substr(0, prefix.size()), prefix);
  EXPECT_EQ(published.substr(published.find("layers:")), rendered.substr(rendered.find("layers:")));
}

// 1e+06 would load as a string in YAML 1.1; the budget must read back as the number written.
TEST(RunManifestTest, WritesTheCostBudgetAsAFullPrecisionNumber) {
  RunManifest manifest;
  manifest.columns = referenceColumns();
  for (const double budget : {1000000.0, 1234.5678}) {
    SCOPED_TRACE(budget);
    manifest.cost_budget = budget;
    const std::string yaml = manifest.serialize();
    const std::string key = "\ncost_budget: ";
    const std::size_t start = yaml.find(key);
    ASSERT_NE(start, std::string::npos);
    const std::size_t value_start = start + key.size();
    const std::string value = yaml.substr(value_start, yaml.find('\n', value_start) - value_start);
    EXPECT_EQ(value.find_first_not_of("0123456789."), std::string::npos) << value;
    EXPECT_EQ(std::stod(value), budget);
  }
}

TEST(RunManifestTest, OmitsUnsetOptionalFields) {
  RunManifest manifest;
  manifest.columns = referenceColumns();
  const std::string path = tmpPath("run_manifest_minimal.yaml");
  ::unlink(path.c_str());
  ASSERT_TRUE(manifest.write(path));
  const std::string yaml = readFile(path);
  EXPECT_EQ(yaml.find("cost_budget:"), std::string::npos);
  EXPECT_EQ(yaml.find("random_seed:"), std::string::npos);
}

// A written row must be on disk even if the process dies without unwinding.
// SIGKILL cannot be caught, so no destructor/flush runs — only writeRow()'s own
// flush can save the data. (SIGINT would prove nothing: normal teardown flushes.)
TEST(PerformanceLogTest, RowSurvivesSigkill) {
  const std::string path = tmpPath("crash.csv");
  ::unlink(path.c_str());

  pid_t pid = fork();
  ASSERT_NE(pid, -1);
  if (pid == 0) {
    HarnessMetrics metrics;
    PerformanceLog log;
    log.addMetrics(metrics);
    log.open(path);
    metrics.a = 42.0;
    metrics.b = 5;
    log.writeRow();
    ::kill(::getpid(), SIGKILL);  // hard crash: no dtor, no explicit close/flush
    _exit(0);                     // unreachable
  }

  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, 0), pid);
  ASSERT_TRUE(WIFSIGNALED(status));
  ASSERT_EQ(WTERMSIG(status), SIGKILL);

  const std::string content = readFile(path);
  EXPECT_NE(content.find("A,B"), std::string::npos);
  EXPECT_NE(content.find("42"), std::string::npos);
}

}  // namespace
}  // namespace open_apero
