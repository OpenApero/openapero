#include "open_apero_core/plugin/plugin_library.h"

#include <dlfcn.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "open_apero_core/plugin/plugin_registry.h"

namespace open_apero {

void loadPluginLibrary(const std::string& library) {
  const std::size_t num_duplicates_before = PluginRegistry::duplicateNames().size();
  // RTLD_NOW: an unresolved symbol fails here, with its name, not at the first call into the
  // library. RTLD_LOCAL: the library's own symbols stay out of the global scope; its registrations
  // reach the registry through the core library it links, so nothing has to look them up.
  void* handle = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (handle == nullptr) {
    const char* reason = dlerror();
    throw std::runtime_error("loadPluginLibrary: cannot load '" + library + "': " + (reason != nullptr ? reason : "unknown error"));
  }
  // Never dlclose(): the registry holds constructors defined in the library.
  const std::vector<std::string>& duplicates = PluginRegistry::duplicateNames();
  if (duplicates.size() > num_duplicates_before) {
    std::string names;
    for (std::size_t i = num_duplicates_before; i < duplicates.size(); ++i) {
      names += (i == num_duplicates_before ? "" : ", ") + duplicates[i];
    }
    throw std::runtime_error("loadPluginLibrary: '" + library + "' registers type names that are already registered: " + names + ".");
  }
}

}  // namespace open_apero
