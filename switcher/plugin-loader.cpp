/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "./plugin-loader.hpp"
#include "./documentation-registry.hpp"

namespace switcher {
PluginLoader::PluginLoader() {}

PluginLoader::~PluginLoader() {
  if (module_ != nullptr) g_module_close(module_);
}

BoolLog PluginLoader::load(const char* filename) {
  if (!g_module_supported()) {
    return BoolLog(false, std::string("g_module not supported !, cannot load ") + filename);
  }
  close();

  module_ = g_module_open(filename, G_MODULE_BIND_LAZY);

  if (!module_) {
    return BoolLog(false, std::string("loading ") + filename + ": " + g_module_error());
  }

  if (!g_module_symbol(module_, "create", (gpointer*)&create_)) {
    close();
    return BoolLog(false, std::string("loading ") + filename + ": " + g_module_error());
  }

  if (create_ == nullptr) {
    close();
    return BoolLog(false, std::string("create is null for ") + filename + ": " + g_module_error());
  }

  if (!g_module_symbol(module_, "destroy", (gpointer*)&destroy_)) {
    close();
    return BoolLog(false, std::string("loading ") + filename + ": " + g_module_error());
  }

  if (destroy_ == nullptr) {
    close();
    return BoolLog(false, std::string("destroy is null for ") + filename + ": " + g_module_error());
  }

  if (!g_module_symbol(module_, "get_quiddity_type", (gpointer*)&get_type_)) {
    close();
    return BoolLog(false, std::string("loading ") + filename + ": " + g_module_error());
  }

  if (get_type_ == nullptr) {
    close();
    return BoolLog(false,
                   std::string("get_type is null for ") + filename + ": " + g_module_error());
  }

  class_name_ = get_type_();
  return BoolLog(true);
}

BoolLog PluginLoader::close() {
  if (module_ == nullptr) return BoolLog(false, "trying to close a a null module");

  if (!g_module_close(module_)) {
    return BoolLog(false, std::string("error when closing module: ") + g_module_error());
  }
  module_ = nullptr;
  return BoolLog(true);
}

std::string PluginLoader::get_class_name() const {
  if (module_ == nullptr) return std::string();
  return class_name_;
}
}
