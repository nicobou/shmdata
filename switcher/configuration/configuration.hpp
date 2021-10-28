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

#ifndef __SWITCHER_CONFIGURATION_H__
#define __SWITCHER_CONFIGURATION_H__

#include <filesystem>
#include <vector>

#include "../infotree/information-tree.hpp"
#include "../logger/logged.hpp"

namespace fs = std::filesystem;

namespace switcher {
class Configuration : public log::Logged {
 public:
  using on_loaded_t = std::function<void()>;
  Configuration(log::Base* log, on_loaded_t on_loaded_cb);
  Configuration() = delete;

  bool from_file(const std::string& file_path);
  InfoTree::ptr get();
  void set(InfoTree::ptr conf);

  /**
   * Retrieves a vector of configured path(s) for the `extraConfig` key.
   *
   * Available configuration keys:
   *
   * "extraConfig": "/path/to/extra_config_file.json"
   *
   *  or
   *
   * "extraConfig": [
   *   "/path/to/extra_config_1.json",
   *   "/path/to/extra_config_2.json",
   * ]
   *
   * @return A vector of any extra configuration paths
   */
  std::vector<fs::path> list_extra_configs();

  /**
   * Reads the content of an extra configuration file
   *
   * @param name The name of the extra configuration file
   *
   * @return The content of an extra configuration file as a string
   */
  std::string get_extra_config(const std::string &name);

  /**
   * Get a value in the configuration
   *
   * @param branch_path The path of the value in the configuration
   *
   * @return the value
   */
  Any get_value(const std::string& branch_path) const;

  /**
   * Set a value in the configuration
   *
   * @param branch_path The path of the value in the configuration
   * @param value The value to set for the key
   *
   * @return A boolean asserting how the update went
   */
  template <typename T>
  bool set_value(const std::string& branch_path, T value) {
    return configuration_->branch_set_value(branch_path, value);
  };

 private:
  on_loaded_t on_loaded_cb_;
  InfoTree::ptr configuration_{};
  static const int kMaxConfigurationFileSize{100000000};  // 100Mo
};
}  // namespace switcher

#endif
