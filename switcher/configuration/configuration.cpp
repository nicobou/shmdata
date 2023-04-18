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

#include <unistd.h>

#include <fstream>
#include <vector>

#include "../infotree/json-serializer.hpp"
#include "../utils/file-utils.hpp"

namespace switcher {

namespace fu = fileutils;

const int Configuration::kMaxConfigurationFileSize = 100000000;  // 100MB

  Configuration::Configuration(std::function<void()> on_reloaded) : on_reloaded_(on_reloaded){
  // intialize default config tree from the `global configuration file`
  const fs::path config_file_path = get_default_global_path();

  // init configuration from file
  if (!from_file(config_file_path)) {
    // set a default config if anything went wrong
    configuration_ = InfoTree::make();
  }
  set_defaults();
}

BoolLog Configuration::from_file(const std::string& file_path) {
  // check for file existance on disk
  if (!fs::is_regular_file(file_path)) {
    return BoolLog(false, "file " + file_path + " does not exist");
  }
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    return BoolLog(false, "cannot open " + file_path + " for loading configuration");
  }
  // get file content into a string
  std::string config;
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    return BoolLog(false, "file " + file_path + " is empty");
  }
  if (size > kMaxConfigurationFileSize) {
    return BoolLog(false, "file " + file_path + " is too large, max is "
		   + std::to_string(kMaxConfigurationFileSize)+ " bytes");
  }
  config.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  config.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  // building the tree
  auto tree = infotree::json::deserialize(config);
  if (!tree) {
    return BoolLog(false, "configuration tree cannot be constructed from " + file_path);
  }

  file_stream.close();

  // writing new configuration
  configuration_ = InfoTree::merge(configuration_.get(), tree.get());

  set_defaults();
  if (on_reloaded_) on_reloaded_();
  return BoolLog(true);
}

bool Configuration::from_tree(const InfoTree* tree) {
  configuration_ = InfoTree::merge(configuration_.get(), tree);
  set_defaults();
  if (on_reloaded_) on_reloaded_();
  return true;
}

void Configuration::set_defaults() {

  // check for default `shm` settings
  const std::string shmdir = get_value(".shm.directory");
  if (shmdir.empty()) {
    auto* xdg_runtime_dir_env = std::getenv("XDG_RUNTIME_DIR");
    const fs::path shmdir_path =
        fs::path(xdg_runtime_dir_env ? std::string(xdg_runtime_dir_env) : std::string("/tmp"));
    set_value(".shm.directory", shmdir_path.c_str());
  }

  const std::string shmprefix = get_value(".shm.prefix");
  if (shmprefix.empty()) set_value(".shm.prefix", "switcher_");
}

const InfoTree::ptr Configuration::get() { return configuration_; }

fs::path Configuration::get_default_global_path() {
  if (getgid()) {
    auto* xdg_config_home_env = std::getenv("XDG_CONFIG_HOME");
    const auto env_xch = xdg_config_home_env ? std::string(xdg_config_home_env) : std::string();
    const auto default_xch = fs::path(std::getenv("HOME")) / ".config";

    // global configuration file path
    return (!env_xch.empty() ? fs::path(env_xch) : default_xch) / "switcher" / "global.json";
  } else {
    return "/etc/switcher/global.json";
  }
}

std::vector<fs::path> Configuration::list_extra_configs() {
  // the list for extra configuration paths
  std::vector<fs::path> extra_configs;
  // check if value for config key is a string
  auto path = configuration_->branch_get_value("extraConfig");
  if (path.is<std::string>()) {
    // append it to extra configs list
    extra_configs.push_back(fs::path(path));
  } else {
    // might be an array
    auto paths = configuration_->copy_leaf_values("extraConfig");
    for (auto& it : paths) {
      // append it to extra configs list
      extra_configs.push_back(fs::path(it));
    };
  }
  return extra_configs;
};

BoolLog Configuration::get_extra_config(const std::string& name) {
  // iterate over the vector of extra configurations
  for (const auto& path : list_extra_configs()) {
    // compare name with configured extra configuration filename
    if (name == path.filename()) return BoolLog(true, fu::get_content(path.c_str()));
  }
  return BoolLog(false, "extra configuration " + name + " was not found");
}

Any Configuration::get_value(const std::string& branch_path) const {
  return configuration_->branch_get_value(branch_path);
}

}  // namespace switcher
