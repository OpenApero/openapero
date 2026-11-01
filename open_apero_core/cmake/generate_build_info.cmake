# Generates open_apero_core/logging/build_info.h with the git provenance of the
# source being compiled. Run at build time (cmake -P) so the SHA reflects the
# working tree, not the configure-time snapshot.
#
# Inputs (via -D): GIT_SRC_DIR (a path inside the repo), OUTPUT_FILE.
#
# One component per repository (this one and any superproject), named after its
# top-level directory. "Dirty" includes untracked, non-ignored files; a dirty tree
# never fails the build.

find_package(Git QUIET)

# Appends "<name>|<sha>|<dirty>" for the repository containing ${dir} to
# ${out_var}, and reports that repository's top level in ${out_toplevel}.
function(_collect_component dir out_var out_toplevel)
  set(${out_toplevel} "" PARENT_SCOPE)

  execute_process(
    COMMAND ${GIT_EXECUTABLE} -C "${dir}" rev-parse --show-toplevel
    OUTPUT_VARIABLE _toplevel
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _toplevel_result)
  if(NOT _toplevel_result EQUAL 0 OR "${_toplevel}" STREQUAL "")
    return()
  endif()

  execute_process(
    COMMAND ${GIT_EXECUTABLE} -C "${_toplevel}" rev-parse HEAD
    OUTPUT_VARIABLE _sha
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _sha_result)
  if(NOT _sha_result EQUAL 0 OR "${_sha}" STREQUAL "")
    set(_sha "unknown")
  endif()

  # Untracked-but-not-ignored files count, whatever status.showUntrackedFiles is set to.
  execute_process(
    COMMAND ${GIT_EXECUTABLE} -C "${_toplevel}" status --porcelain --untracked-files=normal
    OUTPUT_VARIABLE _status
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if("${_status}" STREQUAL "")
    set(_dirty "false")
  else()
    set(_dirty "true")
  endif()

  get_filename_component(_name "${_toplevel}" NAME)
  set(_collected "${${out_var}}")
  list(APPEND _collected "${_name}|${_sha}|${_dirty}")
  set(${out_var} "${_collected}" PARENT_SCOPE)
  set(${out_toplevel} "${_toplevel}" PARENT_SCOPE)
endfunction()

set(COMPONENTS "")
if(Git_FOUND)
  _collect_component("${GIT_SRC_DIR}" COMPONENTS _current)

  # Walk out through every enclosing superproject, so a submodule of a submodule
  # still reports the revision of the outermost tree that was actually built.
  set(_depth 0)
  while(NOT "${_current}" STREQUAL "" AND _depth LESS 8)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} -C "${_current}" rev-parse --show-superproject-working-tree
      OUTPUT_VARIABLE _super
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if("${_super}" STREQUAL "")
      break()
    endif()
    _collect_component("${_super}" COMPONENTS _current)
    math(EXPR _depth "${_depth} + 1")
  endwhile()
endif()

if("${COMPONENTS}" STREQUAL "")
  # No git, or not a repository: say so rather than implying a clean build.
  set(COMPONENTS "unknown|unknown|true")
endif()

# The first component is the repository owning this file. kGitSha/kGitDirty stay
# as its aliases so a consumer wanting "this library's revision" need not index.
list(GET COMPONENTS 0 _primary)
string(REPLACE "|" ";" _primary_fields "${_primary}")
list(GET _primary_fields 1 PRIMARY_SHA)
list(GET _primary_fields 2 PRIMARY_DIRTY)

set(_entries "")
foreach(_component IN LISTS COMPONENTS)
  string(REPLACE "|" ";" _fields "${_component}")
  list(GET _fields 0 _name)
  list(GET _fields 1 _sha)
  list(GET _fields 2 _dirty)
  string(APPEND _entries "    {\"${_name}\", \"${_sha}\", ${_dirty}},\n")
endforeach()

set(_content "#pragma once
// Generated at build time by generate_build_info.cmake. Do not edit.
#include <cstddef>

namespace open_apero {
namespace build_info {

/// One repository that contributed source to this build.
struct Component {
  const char* name;  ///< Repository top-level directory name.
  const char* sha;   ///< HEAD commit, or \"unknown\".
  bool dirty;        ///< Tracked modifications, or untracked non-ignored files.
};

/// Innermost repository first, then each enclosing superproject.
inline constexpr Component kComponents[] = {
${_entries}};
inline constexpr std::size_t kNumComponents = sizeof(kComponents) / sizeof(kComponents[0]);

/// Revision of the repository owning this library, i.e. kComponents[0].
inline constexpr const char* kGitSha = \"${PRIMARY_SHA}\";
inline constexpr bool kGitDirty = ${PRIMARY_DIRTY};

}  // namespace build_info
}  // namespace open_apero
")

# Content-stable write: rewrite only when the content changes, so the core
# library does not relink on every build.
set(_existing "")
if(EXISTS "${OUTPUT_FILE}")
  file(READ "${OUTPUT_FILE}" _existing)
endif()
if(NOT "${_existing}" STREQUAL "${_content}")
  file(WRITE "${OUTPUT_FILE}" "${_content}")
endif()
