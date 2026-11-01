#pragma once

/**
 * @file plugin_library.h
 * @brief Loading plugin libraries at runtime.
 */

#include <string>

namespace open_apero {

/**
 * @brief Load a shared library so the plugins it holds register themselves.
 *
 * Loading runs the library's static initializers, and with them every
 * PluginRegistry::Registration it holds. The library is never unloaded, since the
 * registry keeps constructors that live in it. A library that is already loaded, by
 * the executable's link line or an earlier call, registers nothing again. The name is
 * resolved as dlopen() does: an absolute path as given, a bare file name through the
 * loader's search path (LD_LIBRARY_PATH, the executable's RPATH, the system directories).
 *
 * @param library File name or path of the shared library.
 * @throws std::runtime_error if the library cannot be loaded (with the loader's reason),
 *         or if it registers a type name that was already registered.
 */
void loadPluginLibrary(const std::string& library);

}  // namespace open_apero
