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

#ifndef __SWITCHER_QUIDDITY_FACTORY_H__
#define __SWITCHER_QUIDDITY_FACTORY_H__

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../infotree/information-tree.hpp"
#include "../logger/logger.hpp"
#include "../utils/abstract-factory.hpp"
#include "./config.hpp"
#include "./plugin-loader.hpp"

namespace switcher {
namespace quiddity {
class Factory {
  friend class Container;
 public:
  Factory(logger::Logger* logger);
  Factory() = delete;
  std::vector<std::string> get_plugin_dirs() const;
  std::string get_default_plugin_dir() const;
  std::vector<std::string> get_kinds() const;
  InfoTree::ptr get_kinds_doc() const;
  bool exists(const std::string& kind) const;
  bool scan_dir(const std::string& directory_path);
  void register_kind_with_custom_factory(const std::string& kind,
                                         Quiddity* (*custom_create)(quiddity::Config&&),
                                         void (*custom_destroy)(Quiddity*));

 private:
  const logger::Logger* logger_;
  // create is private because it must be called from quiddity container only
  std::shared_ptr<Quiddity> create(const std::string& kind, quiddity::Config&& config);
  bool load_plugin(const std::string& filename);
  void close_plugin(const std::string& kind);

  std::vector<std::string> plugin_dirs_{};
  AbstractFactory<Quiddity, std::string, quiddity::Config&&> abstract_factory_{};
  std::unordered_map<std::string, PluginLoader::uptr> plugins_{};
};

}  // namespace quiddity
}  // namespace switcher
#endif
