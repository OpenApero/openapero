#pragma once

/**
 * @file seeding.h
 * @brief Reproducible per-instance RNG seeding.
 */

#include <cstdint>
#include <random>
#include <string>
#include <string_view>

#include "open_apero_core/common/hashing.h"

namespace open_apero {

/**
 * @brief Build a Mersenne-Twister engine for one stochastic plugin instance.
 *
 * With @p random_seed < 0 the engine is seeded nondeterministically. Otherwise the seed is
 * derived from @p random_seed and @p instance_ns (FNV-1a), so runs reproduce and instances of
 * one plugin type draw decorrelated streams.
 */
inline std::mt19937 makeSeededEngine(int random_seed, std::string_view instance_ns) {
  if (random_seed < 0) {
    std::random_device rd;
    return std::mt19937(rd());
  }
  std::seed_seq seq{static_cast<std::uint32_t>(random_seed), fnv1a32(instance_ns)};
  return std::mt19937(seq);
}

/**
 * @brief A persistent per-plugin RNG that reproduces across episodes.
 *
 * Hold one as a member of a stochastic plugin: call seed() once from
 * setupFromParamMap (passing the plugin's "param_namespace") and reseed() from
 * resetEpisode(), then draw from `engine`.
 */
struct SeededRng {
  std::mt19937 engine;
  std::string instance_ns;

  void seed(int random_seed, const std::string& ns) {
    instance_ns = ns;
    engine = makeSeededEngine(random_seed, instance_ns);
  }

  void reseed(int random_seed) { engine = makeSeededEngine(random_seed, instance_ns); }
};

}  // namespace open_apero
