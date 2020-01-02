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

#include "./configuration.hpp"
#include <fstream>
#include "../infotree/information-tree-json.hpp"

namespace switcher {
Configuration::Configuration(log::BaseLogger* log, on_loaded_t on_loaded_cb)
    : log::Logged(log), on_loaded_cb_(on_loaded_cb) {}

bool Configuration::from_file(const std::string& file_path) {
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    warning("cannot open % for loading configuration", file_path);
    return false;
  }
  // get file content into a string
  std::string config;
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    warning("file % is empty", file_path);
    return false;
  }
  if (size > kMaxConfigurationFileSize) {
    warning("file % is too large, max is % bytes",
            file_path,
            std::to_string(kMaxConfigurationFileSize));
    return false;
  }
  config.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  config.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  // building the tree
  auto tree = infotree::json::deserialize(config);
  if (!tree) {
    warning("configuration tree cannot be constructed from file %", file_path);
    return false;
  }
  // writing new configuration
  configuration_ = tree;

  on_loaded_cb_();
  return true;
}

InfoTree::ptr Configuration::get() { return configuration_; }

void Configuration::set(InfoTree::ptr conf) { configuration_ = conf; }
}  // namespace switcher
