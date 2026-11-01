#pragma once

/**
 * @file hashing.h
 * @brief Implementation-independent hashing for reproducible seeding.
 */

#include <cstdint>
#include <string_view>

namespace open_apero {

/**
 * @brief 32-bit FNV-1a hash of a byte string.
 *
 * Fully specified, unlike std::hash, so seeds derived from it reproduce on every platform.
 */
inline uint32_t fnv1a32(std::string_view data) {
  uint32_t hash = 0x811c9dc5u;  // FNV offset basis
  for (char c : data) {
    hash ^= static_cast<uint32_t>(static_cast<unsigned char>(c));
    hash *= 0x01000193u;  // FNV prime
  }
  return hash;
}

}  // namespace open_apero
