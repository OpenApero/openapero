#pragma once

/**
 * @file schema_fingerprint.h
 * @brief Deterministic 64-bit fingerprint of a performance-log column schema.
 *
 * Byte-compatible with open_apero_analysis/fingerprint.py so a consumer can
 * recompute a run's fingerprint from its manifest and confirm the schema.
 *
 * Canonical encoding: the schema is the ordered column list; each column
 * contributes four fields — name, type, unit, qualified_parent — each encoded
 * as a 4-byte big-endian length followed by its UTF-8 bytes. The length prefix
 * makes ':' or newlines in any field unable to forge a delimiter. The byte
 * stream is hashed with 64-bit FNV-1a and rendered as 16 lowercase hex digits.
 * std::hash is deliberately NOT used (not a portable, stable contract).
 */

#include <cstdint>
#include <string>
#include <vector>

#include "open_apero_core/logging/performance_log.h"

namespace open_apero {

/// 64-bit FNV-1a constants (spelled out identically in the Python implementation).
constexpr uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

/// 64-bit FNV-1a hash of a byte string.
uint64_t fnv1a64(const std::string& data);

/// Lowercase-hex 64-bit fingerprint of an ordered column schema.
std::string schemaFingerprint(const std::vector<PerformanceLog::Column>& columns);

}  // namespace open_apero
