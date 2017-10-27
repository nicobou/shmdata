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

#ifndef __SWITCHER_QUIDDITY_CONTAINER_H__
#define __SWITCHER_QUIDDITY_CONTAINER_H__

#include <memory>
#include <thread>
#include <unordered_map>

#include "./abstract-factory.hpp"
#include "./documentation-registry.hpp"
#include "./information-tree.hpp"
#include "./logged.hpp"
#include "./plugin-loader.hpp"
#include "./quiddity-configuration.hpp"
#include "./quiddity.hpp"

namespace switcher {
class QuidditySignalSubscriber;
class Switcher;
class Bundle;

class QuiddityContainer : public Logged {
  friend class Bundle;

 public:
  using ptr = std::shared_ptr<QuiddityContainer>;
  using OnCreateRemoveCb = std::function<void(const std::string& nick_name)>;

  static QuiddityContainer::ptr make_container(Switcher* switcher,
                                               BaseLogger* log,
                                               const std::string& name = "default");
  QuiddityContainer() = delete;
  virtual ~QuiddityContainer() = default;
  QuiddityContainer(const QuiddityContainer&) = delete;
  QuiddityContainer& operator=(const QuiddityContainer&) = delete;

  // plugins
  bool scan_directory_for_plugins(const std::string& directory_path);
  std::vector<std::string> get_plugin_dirs() const;

  // configuration
  bool load_configuration_file(const std::string& file_path);

  // **** info
  std::string get_name() const;
  std::vector<std::string> get_classes();
  std::vector<std::string> get_instances() const;
  bool has_instance(const std::string& name) const;

  // doc
  std::string get_classes_doc();                                           // json formatted
  std::string get_class_doc(const std::string& class_name);                // json formatted
  std::string get_quiddity_description(const std::string& name);           // json formatted
  std::string get_quiddities_description();                                // json formatted
  bool class_exists(const std::string& class_name);

  // **** creation/remove/get and notification
  std::string create(const std::string& quiddity_class, bool call_creation_cb = true);
  std::string create(const std::string& quiddity_class,
                     const std::string& name,
                     bool call_creation_cb = true);
  bool remove(const std::string& name, bool call_removal_cb = true);
  std::shared_ptr<Quiddity> get_quiddity(const std::string& name);

  unsigned int register_creation_cb(OnCreateRemoveCb cb);
  unsigned int register_removal_cb(OnCreateRemoveCb cb);
  void unregister_creation_cb(unsigned int id);
  void unregister_removal_cb(unsigned int id);
  void reset_create_remove_cb();

  // information tree
  Forward_consultable_from_associative_container(
      QuiddityContainer,       // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      std::string,             // key type for accessor
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      tree,                    // method used by quiddities to access the consultable
      use_tree);               // public forwarding method

  Forward_delegate_from_associative_container(
      QuiddityContainer,       // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      std::string,             // key type for accessor
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      user_data,               // method used by quiddities to access the consultable
      user_data);              // public forwarding method

  // **** properties
  Forward_consultable_from_associative_container(
      QuiddityContainer,       // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      std::string,             // accessor key type
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      prop,                    // method used by quiddities to access the consultable
      props);                  // public forwarding method

  // **** methods
  // doc (json formatted)
  std::string get_methods_description(const std::string& quiddity_name);
  std::string get_method_description(const std::string& quiddity_name,
                                     const std::string& method_name);
  // following "by_class" methods provide properties available after creation
  // only
  std::string get_methods_description_by_class(const std::string& class_name);
  std::string get_method_description_by_class(const std::string& class_name,
                                              const std::string& method_name);
  // invoke
  bool invoke(const std::string& quiddity_name,
              const std::string& method_name,
              std::string** return_value,
              const std::vector<std::string>& args);

  bool has_method(const std::string& quiddity_name, const std::string& method_name);

  // **** signals
  Forward_consultable_from_associative_container(
      QuiddityContainer,       // self type
      Quiddity,                // consultable type
      find_quiddity,           // accessor
      std::string,             // accessor key type
      construct_error_return,  // what is suposed to be returned when key has
                               // not been found
      sig,                     // method used by quiddities to access the consultable
      sigs);                   // public forwarding method

  Switcher* get_switcher() { return switcher_; };

 private:
  bool load_plugin(const char* filename);
  void close_plugin(const std::string& class_name);
  QuiddityContainer(const std::string&, BaseLogger* log);
  void make_classes_doc();
  void register_classes();
  void remove_shmdata_sockets();
  void release_g_error(GError* error);

  // forwarding accessor and return constructor on error
  std::pair<bool, Quiddity*> find_quiddity(const std::string& name) const {
    auto it = quiddities_.find(name);
    if (quiddities_.end() == it) {
      return std::make_pair(false, nullptr);
    }
    return std::make_pair(true, it->second.get());
  };
  // construct result to pass when element has not been found:
  template <typename ReturnType>
  ReturnType construct_error_return(const std::string& name) const {
    warning("quiddity % not found", name);
    return ReturnType();
  }

  std::vector<std::string> plugin_dirs_{};
  InfoTree::ptr configurations_{};
  std::unordered_map<std::string, PluginLoader::ptr> plugins_{};
  std::string name_{};
  AbstractFactory<Quiddity, std::string, QuiddityConfiguration&&> abstract_factory_{};
  static const int kMaxConfigurationFileSize{100000000};  // 100Mo

  std::map<unsigned int, OnCreateRemoveCb> on_created_cbs_{};
  std::map<unsigned int, OnCreateRemoveCb> on_removed_cbs_{};
  std::unordered_map<std::string, std::shared_ptr<QuidditySignalSubscriber>> signal_subscribers_{};
  InfoTree::ptr classes_doc_{};
  CounterMap counters_{};
  std::weak_ptr<QuiddityContainer> me_{};
  Switcher* switcher_{nullptr};
  std::unordered_map<std::string, std::shared_ptr<Quiddity>> quiddities_{};
};

}  // namespace switcher
#endif
