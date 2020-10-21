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

#include <dlfcn.h>

#include "./documentation-registry.hpp"

namespace switcher {
namespace quiddity {

PluginLoader::~PluginLoader() {
  if (module_ == nullptr) return;
  dlclose(module_);
}

PluginLoader::PluginLoader(const std::string& filename)
    : BoolLog(true), module_(dlopen(filename.c_str(), RTLD_LAZY | RTLD_GLOBAL)) {
  if (!module_) {
    is_valid_ = false;
    msg_ = std::string("loading ") + filename + ": " + dlerror();
    return;
  }

  dlerror();  // clear any existing error

  *(void**)(&create_) = dlsym(module_, "create");
  if (!create_) {
    is_valid_ = false;
    msg_ = std::string("loading ") + filename + ": " + ": create symbol not found";
    return;
  }

  *(void**)(&destroy_) = dlsym(module_, "destroy");
  if (!destroy_) {
    is_valid_ = false;
    msg_ = std::string("loading ") + filename + ": " + ": destroy symbol not found";
    return;
  }

  *(void**)(&get_type_) = dlsym(module_, "get_quiddity_type");
  if (!get_type_) {
    is_valid_ = false;
    msg_ = std::string("loading ") + filename + ": " + ": get_quiddity_type symbol not found";
    return;
  }

  dlerror();  // clear any existing error

  class_name_ = get_type_();
}

std::string PluginLoader::get_class_name() const {
  if (module_ == nullptr) return std::string();
  return class_name_;
}

}  // namespace quiddity
}  // namespace switcher
