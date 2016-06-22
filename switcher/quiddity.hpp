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
 * The Quiddity class
 */

#ifndef __SWITCHER_QUIDDITY_H__
#define __SWITCHER_QUIDDITY_H__

#include <gst/gst.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "./gobject-wrapper.hpp"
#include "./information-tree.hpp"
#include "./json-builder.hpp"
#include "./make-consultable.hpp"
#include "./method.hpp"
#include "./property-container.hpp"
#include "./quiddity-documentation.hpp"
#include "./signal-string.hpp"

namespace switcher {
class QuiddityManager_Impl;

class Quiddity {
  // FIXME do something for this (to many friend class in quiddity.hpp):
  friend class StartableQuiddity;
  friend class ShmdataConnector;
  friend class ShmdataWriter;
  friend class ShmdataFollower;
  friend class GstPixelFormatConverter;
  friend class GstVideoCodec;
  friend class GstAudioCodec;
  friend class ShmdataDecoder;

 public:
  typedef std::shared_ptr<Quiddity> ptr;
  Quiddity();
  Quiddity(const Quiddity&) = delete;
  Quiddity& operator=(const Quiddity&) = delete;
  virtual ~Quiddity();

  // class documentation
  virtual QuiddityDocumentation* get_documentation() = 0;

  // class initialisation
  virtual bool init() = 0;

  // instance name
  std::string get_name() const;
  // FIXME name should be a ctor arg
  bool set_name(const std::string& name);  // can be called once

  // FIXME configuration should be a ctor arg
  void set_configuration(InfoTree::ptr config);

  // properties
  Make_consultable(Quiddity, PContainer, &props_, prop);

  // methods
  std::string get_method_description(const std::string& method_name);
  std::string get_methods_description();
  bool invoke_method(const std::string& function_name,
                     std::string** return_value,
                     const std::vector<std::string>& args);
  int method_get_num_pointer_args(
      const std::string& function_name);  // returns -1 if method not found
  bool has_method(const std::string& method_name);

  // signals
  std::string get_signals_description();
  std::string get_signal_description(const std::string& signal_name);
  bool subscribe_signal(const std::string& name, Signal::OnEmittedCallback cb, void* user_data);
  bool unsubscribe_signal(const std::string& name, Signal::OnEmittedCallback cb, void* user_data);
  // information
  template <typename R>
  R invoke_info_tree(std::function<R(InfoTree::ptrc tree)> fun) {
    return fun(information_tree_.get());
  }

  Make_consultable(Quiddity, InfoTree, information_tree_.get(), tree);

  // user data
  Make_delegate(Quiddity, InfoTree, structured_user_data_.get(), user_data);
  Selective_hook(user_data,
                 decltype(&InfoTree::graft),
                 &InfoTree::graft,
                 &Quiddity::user_data_graft_hook);
  Selective_hook(user_data,
                 decltype(&InfoTree::prune),
                 &InfoTree::prune,
                 &Quiddity::user_data_prune_hook);

  // shmdata socket names
  static std::string get_socket_name_prefix();
  static std::string get_socket_dir();

  // manager_impl initialization
  void set_manager_impl(std::shared_ptr<QuiddityManager_Impl> manager_impl);

  // use a consistent naming for shmdatas
  std::string make_file_name(const std::string& suffix) const;
  std::string get_manager_name();
  std::string get_quiddity_name_from_file_name(const std::string& shmdata_path) const;
  std::string get_file_name_prefix() const;

 private:
  // tree used by quiddity to communicate info to user,
  // read-only by user, read/write by quiddity
  InfoTree::ptr information_tree_;

  // writable tree for custom user data, should not be used by quiddity
  // (hooks are installed for signaling graft and prune)
  InfoTree::ptr structured_user_data_;

  // configuration tree. When manager is loaded with a config file,
  // the branch of the tree corresponding to the quiddity type
  // is given to the quiddity. There is no constrain about how quiddity
  // sjhould use this configuration
  InfoTree::ptr configuration_tree_;

  // properties
  PContainer props_;

  // methods
  std::unordered_map<std::string, Method::ptr> methods_{};
  std::unordered_map<std::string, Method::ptr> disabled_methods_{};
  JSONBuilder::ptr methods_description_;

  // position weight
  gint position_weight_counter_{0};

  // pair is <class_name, signal_name>
  // this map is static in order to avoid re-creation of the same signal
  // for each quiddity instance
  static std::map<std::pair<std::string, std::string>, guint> signals_ids_;
  std::unordered_map<std::string, Signal::ptr> signals_{};
  JSONBuilder::ptr signals_description_;

  // naming
  std::string name_{};

  std::mutex self_destruct_mtx_{};

  // user data hooks
  bool user_data_graft_hook(const std::string& path, InfoTree::ptr tree);
  InfoTree::ptr user_data_prune_hook(const std::string& path);

  // position weight
  bool compare_properties(const std::string& first, const std::string& second);

  // method
  bool method_is_registered(const std::string& method_name);
  bool register_method(const std::string& method_name,
                       Method::method_ptr method,
                       Method::return_type return_type,
                       Method::args_types arg_types,
                       gpointer user_data);
  bool set_method_description(const std::string& long_name,
                              const std::string& method_name,
                              const std::string& short_description,
                              const std::string& return_description,
                              const Method::args_doc& arg_description);

  // allows for creation of signals in a parent class (like segment)
  bool make_custom_signal_with_class_name(
      const std::string& class_name,   // quiddity class name that is making the signal
      const std::string& signal_name,  // the name to give
      GType return_type,
      guint n_params,  // number of params
      GType* param_types);

  bool set_signal_description(const std::string& long_name,
                              const std::string& signal_name,
                              const std::string& short_description,
                              const std::string& return_description,
                              const Signal::args_doc& arg_description);

 protected:
  // information
  bool graft_tree(const std::string& path, InfoTree::ptr tree_to_graft, bool do_signal = true);
  InfoTree::ptr prune_tree(const std::string& path, bool do_signal = true);

  // property
  Make_delegate(Quiddity, PContainer, &props_, pmanage);

  // configuration
  Make_consultable(Quiddity, InfoTree, configuration_tree_.get(), config);

  // methods
  bool install_method(const std::string& long_name,
                      const std::string& method_name,
                      const std::string& short_description,
                      const std::string& return_description,
                      const Method::args_doc& arg_description,
                      Method::method_ptr method,
                      Method::return_type return_type,
                      Method::args_types arg_types,
                      gpointer user_data);
  bool disable_method(const std::string& name);
  bool enable_method(const std::string& name);

  // signals
  bool install_signal(const std::string& long_name,
                      const std::string& signal_name,
                      const std::string& short_description,
                      const Signal::args_doc& arg_description,
                      guint number_of_params,
                      GType* param_types);

  bool install_signal_with_class_name(const std::string& class_name,
                                      const std::string& long_name,
                                      const std::string& signal_name,
                                      const std::string& short_description,
                                      const Signal::args_doc& arg_description,
                                      guint number_of_params,
                                      GType* param_types);
  void signal_emit(std::string signal_name, ...);

  // custom signals
  void emit_on_interface_changed();  // in order to tell properties/methods has
                                     // changed

  void self_destruct();

  // used in order to dynamically create other quiddity, weak_ptr is used in
  // order to
  // avoid circular references to the manager_impl
  std::weak_ptr<QuiddityManager_Impl> manager_impl_{};
  std::string manager_name_{};

  // gobject wrapper for custom signals
  GObjectWrapper::ptr gobject_;
};

#define SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(                                           \
    cpp_quiddity_class, class_name, name, category, tags, description, license, author) \
  QuiddityDocumentation cpp_quiddity_class::switcher_doc_(                              \
      name, class_name, category, tags, description, license, author);                  \
  QuiddityDocumentation* cpp_quiddity_class::get_documentation() { return &switcher_doc_; }

#define SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(cpp_quiddity_class) \
  typedef std::shared_ptr<cpp_quiddity_class> ptr;                   \
  QuiddityDocumentation* get_documentation();                        \
  static QuiddityDocumentation switcher_doc_;

#define SWITCHER_DECLARE_PLUGIN(cpp_quiddity_class)                                             \
  extern "C" Quiddity* create(const std::string& name) { return new cpp_quiddity_class(name); } \
  extern "C" void destroy(Quiddity* quiddity) { delete quiddity; }                              \
  extern "C" QuiddityDocumentation* get_documentation() {                                       \
    return &cpp_quiddity_class::switcher_doc_;                                                  \
  }

}  // namespace switcher
#endif
