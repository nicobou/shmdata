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

#include "../infotree/json-serializer.hpp"
#include "../utils/file-utils.hpp"

namespace switcher {

namespace fu = fileutils;

Configuration::Configuration(log::Base* log, on_loaded_t on_loaded_cb)
    : log::Logged(log), on_loaded_cb_(on_loaded_cb) {
  // intialize default config tree from the `global configuration file`
  const auto env_xch = std::getenv("XDG_CONFIG_HOME");
  const auto default_xch = fs::path(std::getenv("HOME")) / ".config"; 
  // global configuration file path
  const fs::path global_config = \
    (env_xch ? fs::path(env_xch) : default_xch) / "switcher" / "global.json";
  // load configuration from file
  if(!this->from_file(global_config)) 
    // set an empty config if anything went wrong 
    this->set(InfoTree::make());
}

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

std::vector<fs::path> Configuration::list_extra_configs() {
  // the list for extra configuration paths
  std::vector<fs::path> extra_configs;
  // retrieve raw pointer to current configuration instance
  const auto config_tree = this->get();
  // check if value for config key is a string
  auto path = config_tree->branch_get_value("extraConfig");
  if (path.is<std::string>()) {
    // append it to extra configs list
    extra_configs.push_back(fs::path(path));
  } else {
    // might be an array
    auto paths = config_tree->copy_leaf_values("extraConfig");
    for (auto& it : paths) {
      // append it to extra configs list
      extra_configs.push_back(fs::path(it));
    };
  }
  return extra_configs;
};

std::string Configuration::get_extra_config(const std::string &name) {
  // iterate over the vector of extra configurations
  for (const auto& path : this->list_extra_configs()) {
    // compare name with configured extra configuration filename
    if(name == path.filename())
      // read and return content
      return fu::get_content(path.c_str());
  }
  warning("Extra configuration `%` was not found", name);
  // extra configuration identified by `name` was not found
  return std::string();
}

}  // namespace switcher
