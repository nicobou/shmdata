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
#include "./quiddity-documentation.hpp"

namespace switcher {
PluginLoader::PluginLoader() {
}

PluginLoader::~PluginLoader() {
  if (module_ != nullptr)
    g_module_close(module_);
}

bool PluginLoader::load(const char *filename) {
  if (!g_module_supported()) {
    g_debug("g_module not supported !, cannot load %s", filename);
    return false;
  }
  close();

  module_ = g_module_open(filename, G_MODULE_BIND_LAZY);

  if (!module_) {
    g_debug("loading %s: %s", filename, g_module_error());
    return false;
  }

  if (!g_module_symbol(module_, "create", (gpointer *) &create_)) {
    g_debug("loading %s: %s", filename, g_module_error());
    close();
    return false;
  }

  if (create_ == nullptr) {
    g_debug("%s: %s", filename, g_module_error());
    close();
    return false;
  }

  if (!g_module_symbol(module_, "destroy", (gpointer *) &destroy_)) {
    g_debug("%s: %s", filename, g_module_error());
    close();
    return false;
  }

  if (destroy_ == nullptr) {
    g_debug("%s: %s", filename, g_module_error());
    close();
    return false;
  }

  if (!g_module_symbol
      (module_, "get_documentation", (gpointer *) &get_documentation_)) {
    g_debug("%s: %s", filename, g_module_error());
    close();
    return false;
  }

  if (get_documentation_ == nullptr) {
    g_debug("%s: %s", filename, g_module_error());
    close();
    return false;
  }

  QuiddityDocumentation *doc = get_documentation_();
  class_name_ = doc->get_class_name();
  //json_doc_ = doc.get_json_root_node();
  return true;
}

bool PluginLoader::close() {
  if (module_ == nullptr)
    return false;

  if (!g_module_close(module_)) {
    g_debug("closing module: %s", g_module_error());
    return false;
  }
  module_ = nullptr;
  return true;
}

std::string PluginLoader::get_class_name() const {
  if (module_ == nullptr)
    return std::string();
  return class_name_;
}

JSONBuilder::Node PluginLoader::get_json_root_node() {
  if (module_ == nullptr)
    return nullptr;
  return get_documentation_()->get_json_root_node();
}

QuiddityDocumentation *PluginLoader::get_doc() {
  return get_documentation_();
}
}
