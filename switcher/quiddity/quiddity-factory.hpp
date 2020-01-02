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
#include "../logger/logged.hpp"
#include "../utils/abstract-factory.hpp"
#include "./plugin-loader.hpp"
#include "./quiddity-configuration.hpp"

namespace switcher {
namespace quid {
class Factory : public log::Logged {
  friend class Container;

 public:
  Factory(log::BaseLogger* log);
  std::vector<std::string> get_plugin_dirs() const;
  std::string get_default_plugin_dir() const;
  std::vector<std::string> get_class_list() const;
  InfoTree::ptr get_classes_doc() const;
  bool exists(const std::string& class_name) const;
  bool scan_dir(const std::string& directory_path);
  void register_class_with_custom_factory(const std::string& class_name,
                                          Quiddity* (*custom_create)(quid::Config&&),
                                          void (*custom_destroy)(Quiddity*));

 private:
  // create is private because it must be called from quiddity container only
  std::shared_ptr<Quiddity> create(const std::string& class_name, quid::Config&& config);
  bool load_plugin(const std::string& filename);
  void close_plugin(const std::string& class_name);

  std::vector<std::string> plugin_dirs_{};
  AbstractFactory<Quiddity, std::string, quid::Config&&> abstract_factory_{};
  std::unordered_map<std::string, PluginLoader::ptr> plugins_{};
};

}  // namespace quid
}  // namespace switcher
#endif
