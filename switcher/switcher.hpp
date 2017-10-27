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

/**
 * The Switcher class
 */

#ifndef __SWITCHER_SWITCHER_H__
#define __SWITCHER_SWITCHER_H__

#include <stdarg.h>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "./base-logger.hpp"
#include "./console-logger.hpp"
#include "./information-tree.hpp"
#include "./invocation-spec.hpp"
#include "./make-consultable.hpp"
#include "./quiddity-container.hpp"
#include "./switcher-wrapper.hpp"
#include "./threaded-wrapper.hpp"

namespace switcher {
class Switcher {
  friend class Bundle;  // access to qcontainer_
 public:
  using ptr = std::shared_ptr<Switcher>;

  ~Switcher() = default;

  template <typename L = ConsoleLogger, typename... Largs>
  static Switcher::ptr make_switcher(const std::string& name, Largs... args) {
    init_gst();
    Switcher::ptr switcher(new Switcher(name, std::make_unique<L>(std::forward<Largs>(args)...)));
    switcher->me_ = switcher;
    return switcher;
  }

  Switcher& operator=(const Switcher&) = delete;
  Switcher(const Switcher&) = delete;
  std::string get_name() const;

  // switcher state
  // you should use InfoTree json serialization and Fileutils for
  // saving to file
  InfoTree::ptr get_state() const;
  bool load_state(InfoTree::ptr state);
  void reset_state(bool remove_created_quiddities);

  // plugins
  bool scan_directory_for_plugins(const std::string& directory);

  // configuration
  bool load_configuration_file(const std::string& file_path);

  // inspect
  std::vector<std::string> get_classes();           // know which quiddities can be created
  std::vector<std::string> get_quiddities() const;  // know instances
  // doc (json formatted)
  std::string get_classes_doc();
  std::string get_class_doc(const std::string& class_name);
  std::string get_quiddity_description(const std::string& quiddity_name);
  std::string get_quiddities_description();
  // create/remove
  std::string create(const std::string& class_name);
  // &?= chars are not allowed in nicknames
  std::string create(const std::string& class_name, const std::string& nick_name);
  bool remove(const std::string& quiddity_name);
  unsigned int register_creation_cb(QuiddityContainer::OnCreateRemoveCb cb) {
    return qcontainer_->register_creation_cb(cb);
  };
  unsigned int register_removal_cb(QuiddityContainer::OnCreateRemoveCb cb) {
    return qcontainer_->register_removal_cb(cb);
  };
  void unregister_creation_cb(unsigned int id) { qcontainer_->unregister_creation_cb(id); };
  void unregister_removal_cb(unsigned int id) { qcontainer_->unregister_removal_cb(id); };
  void reset_create_remove_cb() { qcontainer_->reset_create_remove_cb(); };

  bool has_quiddity(const std::string& name);
  std::string get_nickname(const std::string& name) const;
  bool set_nickname(const std::string& name, const std::string& nickname);

  // informations
  Forward_consultable(Switcher, QuiddityContainer, qcontainer_.get(), use_tree, use_tree);
  Forward_delegate(Switcher, QuiddityContainer, qcontainer_.get(), user_data, user_data);

  // properties
  Forward_consultable(Switcher, QuiddityContainer, qcontainer_.get(), props, use_prop);

  // signals
  Forward_consultable(Switcher, QuiddityContainer, qcontainer_.get(), sigs, use_sig);

  // methods
  // doc (json formatted)
  std::string get_methods_description(const std::string& quiddity_name);
  std::string get_method_description(const std::string& quiddity_name,
                                     const std::string& method_name);
  // following "by_class" methods provide methods available after creation only
  std::string get_methods_description_by_class(const std::string& class_name);
  std::string get_method_description_by_class(const std::string& class_name,
                                              const std::string& method_name);
  // invoke
  bool invoke(const std::string& quiddity_name,
              const std::string& method_name,
              std::string** return_value,
              const std::vector<std::string>& args);
  bool invoke_va(const std::string& quiddity_name,
                 const std::string& method_name,
                 std::string** return_value,
                 ...);

  bool has_method(const std::string& quiddity_name, const std::string& method_name);

 private:
  // logs
  mutable std::unique_ptr<BaseLogger> log_;
  // invocation of quiddity_qcontainer_ methods in a dedicated thread
  mutable ThreadedWrapper<> invocation_loop_{};
  QuiddityContainer::ptr qcontainer_;  // may be shared with others for
                                       // automatic quiddity creation
  std::string name_;
  std::vector<std::string> quiddities_at_reset_{};
  // gives shared pointer to this:
  std::weak_ptr<Switcher> me_{};
  // invocation
  std::vector<InvocationSpec> invocations_{};

  Switcher() = delete;
  template <typename L>  // unique_ptr<LOG_TYPE>
  Switcher(const std::string& name, L&& log)
      : log_(std::move(log)),
        qcontainer_(QuiddityContainer::make_container(this, log_.get(), name)),
        name_(name) {}
  void auto_init(const std::string& quiddity_name);
  void try_save_current_invocation(const InvocationSpec& invocation_spec);
  static void init_gst();
};
}  // namespace switcher

#endif
