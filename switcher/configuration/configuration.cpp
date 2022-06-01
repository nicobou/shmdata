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

#include <spdlog/sinks/rotating_file_sink.h>
#include <unistd.h>

#include <fstream>
#include <vector>

#include "../infotree/json-serializer.hpp"
#include "../utils/file-utils.hpp"

namespace switcher {

namespace fu = fileutils;

const int Configuration::kMaxConfigurationFileSize = 100000000;  // 100MB

Configuration::Configuration(bool debug, void_func_t _post_load, void_func_t _post_file_sink)
    : post_load(_post_load), post_file_sink(_post_file_sink) {
  // intialize default config tree from the `global configuration file`
  const fs::path config_file_path = this->get_default_global_path();

  // get global switcher logger
  this->logger = spdlog::get("switcher");

  // init configuration from file
  if (!this->from_file(config_file_path))
    // set an empty config if anything went wrong
    this->set(InfoTree::make());

  // check for default `logs` settings
  std::string filepath = this->get_value(".logs.filepath");
  if (filepath.empty()) {
    filepath = std::string(this->get_default_log_path());
    // configure log file path
    this->set_value(".logs.filepath", filepath);
  }

  size_t max_files = this->get_value(".logs.max_files");
  if (!max_files) {
    max_files = 3;
    this->set_value(".logs.max_files", max_files);
  }

  size_t max_size = this->get_value(".logs.max_size");
  if (!max_size) {
    max_size = 1048576 * 100;  // 100MB
    this->set_value(".logs.max_size", max_size);
  } else {
    max_size = 1048576 * max_size;
    this->set_value(".logs.max_size", max_size);
  }

  // append file sink into logger `sinks` vector
  auto file_sink =
      std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filepath, max_size, max_files);
  this->logger->sinks().push_back(file_sink);

  // set logger pattern
  post_file_sink();

  // check log level
  const std::string level_name = this->get_value(".logs.log_level");
  auto level = spdlog::level::from_str(level_name);
  this->logger->set_level(level);

  // debug takes priority over configured log level
  if (debug && level > spdlog::level::debug) {
    this->set_value(".logs.log_level", "debug");
    this->logger->set_level(spdlog::level::debug);
  }

  // check for default `shm` settings
  std::string shmdir = this->get_value(".shm.directory");
  if (shmdir.empty()) {
    auto* xdg_runtime_dir_env = std::getenv("XDG_RUNTIME_DIR");
    const fs::path shmdir_path =
        fs::path(xdg_runtime_dir_env ? std::string(xdg_runtime_dir_env) : std::string("/tmp"));
    this->set_value(".shm.directory", shmdir_path.c_str());
  }

  std::string shmprefix = this->get_value(".shm.prefix");
  if (shmprefix.empty()) this->set_value(".shm.prefix", "switcher_");
}

bool Configuration::from_file(const std::string& file_path) {
  // check for file existance on disk
  if (!fs::is_regular_file(file_path)) {
    LOGGER_DEBUG(this->logger, "file {} does not exist", file_path);
    return false;
  }
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    LOGGER_DEBUG(this->logger, "cannot open {} for loading configuration", file_path);
    return false;
  }
  // get file content into a string
  std::string config;
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    LOGGER_WARN(this->logger, "file {} is empty", file_path);
    return false;
  }
  if (size > kMaxConfigurationFileSize) {
    LOGGER_WARN(this->logger,
                "file {} is too large, max is {:d} bytes",
                file_path,
                kMaxConfigurationFileSize);
    return false;
  }
  config.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  config.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  // building the tree
  auto tree = infotree::json::deserialize(config);
  if (!tree) {
    LOGGER_WARN(this->logger, "configuration tree cannot be constructed from {}", file_path);
    return false;
  }

  file_stream.close();

  // writing new configuration
  configuration_ = tree;

  post_load();
  return true;
}

InfoTree::ptr Configuration::get() { return configuration_; }

void Configuration::set(InfoTree::ptr conf) { configuration_ = conf; }

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

fs::path Configuration::get_default_log_path() {
  if (getgid()) {
    const auto xsh_env = std::getenv("XDG_STATE_HOME");
    const auto xsh_default_path = fs::path(std::getenv("HOME")) / ".local" / "state";
    const auto xsh_dir = fs::path(xsh_env ? std::string(xsh_env) : std::string(xsh_default_path));
    return xsh_dir / "switcher" / "logs" / "switcher.log";
  } else {  // running as root, use default log file path
    return "/var/log/switcher/switcher.log";
  }
}

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

std::string Configuration::get_extra_config(const std::string& name) {
  // iterate over the vector of extra configurations
  for (const auto& path : this->list_extra_configs()) {
    // compare name with configured extra configuration filename
    if (name == path.filename())
      // read and return content
      return fu::get_content(path.c_str());
  }
  LOGGER_WARN(this->logger, "extra configuration {} was not found", name);
  // extra configuration identified by `name` was not found
  return std::string();
}

Any Configuration::get_value(const std::string& branch_path) const {
  return configuration_->branch_get_value(branch_path);
}

}  // namespace switcher
