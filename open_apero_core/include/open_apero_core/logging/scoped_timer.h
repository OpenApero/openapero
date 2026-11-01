#pragma once

/**
 * @file scoped_timer.h
 * @brief RAII helper that accumulates elapsed (monotonic) wall time into a caller-owned double.
 */

#include <chrono>

namespace open_apero {

/**
 * @brief Measures elapsed wall-clock time [s] from construction to destruction and adds it to
 *        a caller-supplied accumulator. A no-op (no clock reads, no writes) when disabled.
 */
class ScopedTimer {
 public:
  ScopedTimer(double& accumulator, bool enabled) : accumulator_(accumulator), enabled_(enabled) {
    if (enabled_) {
      start_ = std::chrono::steady_clock::now();
    }
  }

  ~ScopedTimer() {
    if (enabled_) {
      accumulator_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    }
  }

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

 private:
  double& accumulator_;
  bool enabled_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace open_apero
