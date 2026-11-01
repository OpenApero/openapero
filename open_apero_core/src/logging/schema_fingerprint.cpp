#include "open_apero_core/logging/schema_fingerprint.h"

#include <cstdio>

namespace open_apero {

namespace {

// Append a single field as <4-byte big-endian length><UTF-8 bytes>.
void appendField(std::string& buffer, const std::string& field) {
  const uint32_t length = static_cast<uint32_t>(field.size());
  buffer.push_back(static_cast<char>((length >> 24) & 0xFF));
  buffer.push_back(static_cast<char>((length >> 16) & 0xFF));
  buffer.push_back(static_cast<char>((length >> 8) & 0xFF));
  buffer.push_back(static_cast<char>(length & 0xFF));
  buffer.append(field);
}

}  // namespace

uint64_t fnv1a64(const std::string& data) {
  uint64_t hash = kFnvOffsetBasis;
  for (unsigned char byte : data) {
    hash ^= static_cast<uint64_t>(byte);
    hash *= kFnvPrime;
  }
  return hash;
}

std::string schemaFingerprint(const std::vector<PerformanceLog::Column>& columns) {
  std::string buffer;
  for (const auto& column : columns) {
    appendField(buffer, column.name);
    appendField(buffer, column.type);
    appendField(buffer, column.unit);
    appendField(buffer, column.qualified_parent);
  }
  char hex[17];
  std::snprintf(hex, sizeof(hex), "%016llx", static_cast<unsigned long long>(fnv1a64(buffer)));
  return std::string(hex);
}

}  // namespace open_apero
