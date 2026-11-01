#pragma once

/**
 * @file yaml_file.h
 * @brief YAML scalar formatting and file writing shared by the run manifest and the episode summary.
 *
 * Internal to open_apero_core: not installed.
 */

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

namespace open_apero {
namespace yaml_file {

/// Double-quote a YAML scalar: backslash and double quote are escaped, control characters written as \xNN.
inline std::string yamlQuote(const std::string& value) {
  std::string out = "\"";
  for (const char c : value) {
    const auto code = static_cast<unsigned char>(c);
    if (c == '\\' || c == '"') {
      out.push_back('\\');
      out.push_back(c);
    } else if (code < 0x20 || code == 0x7f) {
      char escaped[5];
      std::snprintf(escaped, sizeof(escaped), "\\x%02x", code);
      out += escaped;
    } else {
      out.push_back(c);
    }
  }
  out.push_back('"');
  return out;
}

/// 15 significant digits; a mantissa without '.' gains ".0" so YAML 1.1 readers load 1e-05 as a float.
inline std::string yamlDouble(double value) {
  if (std::isnan(value)) return ".nan";
  if (std::isinf(value)) return value > 0.0 ? ".inf" : "-.inf";
  std::ostringstream out;
  out << std::setprecision(15) << value;
  std::string text = out.str();
  const std::string::size_type exponent = text.find('e');
  if (exponent != std::string::npos && text.find('.') == std::string::npos) {
    text.insert(exponent, ".0");
  }
  return text;
}

/// Writes the whole buffer, retrying short writes and EINTR.
inline bool writeAll(int fd, const std::string& data) {
  const char* cursor = data.data();
  std::size_t remaining = data.size();
  while (remaining > 0) {
    const ssize_t written = ::write(fd, cursor, remaining);
    if (written < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    cursor += written;
    remaining -= static_cast<std::size_t>(written);
  }
  return true;
}

/// Sets @p error_message, when given, to "<operation> failed: <reason> (errno N)".
inline void setFilesystemError(std::string* error_message, const std::string& operation, int error_number) {
  if (!error_message) return;
  const std::error_code error(error_number, std::generic_category());
  *error_message = operation + " failed: " + error.message() + " (errno " + std::to_string(error_number) + ")";
}

/// Writes @p content to @p path through a temporary file in the same directory, named only once it is complete and
/// on disk: by rename() when @p replace_existing, so a reader sees the old file or the new one, else by link(), which
/// fails rather than overwrite an existing file. The file is readable by everyone and writable by the owner.
inline bool writeFileAtomically(const std::string& path, const std::string& content, bool replace_existing, std::string* error_message) {
  // mkstemp needs a mutable template and creates the file O_EXCL, so concurrent writers never share a temporary.
  std::string temp_path = path + ".XXXXXX";
  const int fd = ::mkstemp(temp_path.data());
  if (fd < 0) {
    setFilesystemError(error_message, "mkstemp", errno);
    return false;
  }

  const auto abandon = [&](const std::string& operation, int error_number) {
    setFilesystemError(error_message, operation, error_number);
    ::close(fd);
    ::unlink(temp_path.c_str());
    return false;
  };

  // mkstemp creates mode 0600.
  constexpr mode_t kFileMode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  if (::fchmod(fd, kFileMode) != 0) {
    return abandon("fchmod", errno);
  }
  if (!writeAll(fd, content)) {
    return abandon("write", errno);
  }
  if (::fsync(fd) != 0) {
    return abandon("fsync", errno);
  }
  if (::close(fd) != 0) {
    const int error_number = errno;
    ::unlink(temp_path.c_str());
    setFilesystemError(error_message, "close", error_number);
    return false;
  }

  const bool named = replace_existing ? std::rename(temp_path.c_str(), path.c_str()) == 0 : ::link(temp_path.c_str(), path.c_str()) == 0;
  const int error_number = errno;
  // rename() consumed the temporary name; link() left it.
  if (!named || !replace_existing) {
    ::unlink(temp_path.c_str());
  }
  if (!named) {
    setFilesystemError(error_message, replace_existing ? "rename" : "link", error_number);
    return false;
  }
  return true;
}

}  // namespace yaml_file
}  // namespace open_apero
