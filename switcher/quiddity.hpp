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

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <map>

#include "./property.hpp"
#include "./method.hpp"
#include "./signal-string.hpp"
#include "./information-tree.hpp"
#include "./quiddity-documentation.hpp"
#include "./json-builder.hpp"
#include "./gobject-wrapper.hpp"
#include "./make-consultable.hpp"

namespace switcher {
class QuiddityManager_Impl;

class Quiddity {
  friend class StartableQuiddity;
  friend class ShmdataConnector;
  friend class ShmdataWriter;
  friend class ShmdataFollower;
  friend class GstPixelFormatConverter;
  friend class GstVideoCodec;
  friend class GstAudioCodec;
  
 public:
  typedef std::shared_ptr<Quiddity> ptr;
  Quiddity();
  Quiddity(const Quiddity &) = delete;
  Quiddity &operator=(const Quiddity &) = delete;
  virtual ~Quiddity();

  // class documentation
  virtual QuiddityDocumentation *get_documentation() = 0;

  // class initialisation
  virtual bool init() = 0;

  // instance name
  const std::string &get_name() const ;
  bool set_name(const std::string &name);  // can be called once

  // properties
  std::string get_property_description(const std::string &property_name);
  std::string get_properties_description();
  bool set_property(const std::string &name,
                    const std::string &value);
  std::string get_property(const std::string &name);
  bool subscribe_property(const std::string &name,
                          Property::Callback cb,
                          void *user_data);
  bool unsubscribe_property(const std::string &name,
                            Property::Callback cb,
                            void *user_data);
  bool has_property(const std::string &property_name);
  Property::ptr get_property_ptr(const std::string &property_name);

  // methods
  std::string get_method_description(const std::string &method_name);
  std::string get_methods_description();
  bool invoke_method(const std::string &function_name,
                     std::string **return_value,
                     const std::vector<std::string> &args);
  int method_get_num_pointer_args(const std::string &function_name);  // returns -1 if method not found
  bool has_method(const std::string &method_name);

  // signals
  std::string get_signals_description();
  std::string get_signal_description(const std::string &signal_name);
  bool subscribe_signal(const std::string &name,
                        Signal::OnEmittedCallback cb,
                        void *user_data);
  bool unsubscribe_signal(const std::string &name,
                          Signal::OnEmittedCallback cb,
                          void *user_data);
  // information
  template <typename R>
  R invoke_info_tree (std::function<R(data::Tree::ptrc tree)> fun) {
    return fun(information_tree_.get());
  }
  // FIXME remove get_info
  std::string get_info(const std::string &path);
  Make_consultable(data::Tree, information_tree_.get(), tree);
    
  // shmdata socket names
  static std::string get_socket_name_prefix();
  static std::string get_socket_dir();

  // manager_impl  initialization
  void set_manager_impl(std::shared_ptr<QuiddityManager_Impl> manager_impl);

 private:
  // information tree
  data::Tree::ptr information_tree_;
  
  // properties
  std::unordered_map<std::string, Property::ptr> properties_{};
  std::unordered_map<std::string, Property::ptr> disabled_properties_{};
  JSONBuilder::ptr properties_description_;

  // methods
  std::unordered_map<std::string, Method::ptr> methods_{};
  std::unordered_map<std::string, Method::ptr> disabled_methods_{};
  bool method_is_registered(const std::string &method_name);
  JSONBuilder::ptr methods_description_;

  // position weight
  gint position_weight_counter_{0};
  bool compare_properties(const std::string &first,
                          const std::string &second);

  // pair is <class_name, signal_name>
  // this map is static in order to avoid re-creation of the same signal
  // for each quiddity instance
  static std::map<std::pair<std::string, std::string>, guint> signals_ids_;
  std::unordered_map<std::string, Signal::ptr> signals_{};
  JSONBuilder::ptr signals_description_;

  // naming
  std::string name_{};

  // property
  bool register_property(GObject *object,
                         GParamSpec *pspec,
                         const std::string &name_to_give,
                         const std::string &long_name,
                         const std::string &signal_to_emit);

  // method
  bool register_method(const std::string &method_name,
                       Method::method_ptr method,
                       Method::return_type return_type,
                       Method::args_types arg_types,
                       gpointer user_data);
  bool set_method_description(const std::string &long_name,
                              const std::string &method_name,
                              const std::string &short_description,
                              const std::string &return_description,
                              const Method::args_doc &arg_description);

  // category and positions
  bool put_method_in_category(const std::string &method,
                              const std::string &category);
  bool set_method_position_weight(const std::string &method,
                                  int position_weight);
  bool put_property_in_category(const std::string &property, const std::string &category);
  bool set_property_position_weight(const std::string &property,
                                    int position_weight);
  
  // signals
  // bool register_signal_gobject(const std::string &signal_name,
  //                              GObject *object,
  //                              const std::string &gobject_signal_name);
  
  // allows for creation of signals in a parent class (like segment)
  bool make_custom_signal_with_class_name(const std::string &class_name,  // quiddity class name that is making the signal
                                          const std::string &signal_name,  // the name to give
                                          GType return_type,
                                          guint n_params,  // number of params
                                          GType *param_types);

  bool set_signal_description(const std::string &long_name,
                              const std::string &signal_name,
                              const std::string &short_description,
                              const std::string &return_description,
                              const Signal::args_doc &arg_description);

 protected:
  // information
  bool graft_tree(const std::string &path,
                  data::Tree::ptr tree_to_graft,
                  bool do_signal = true);
  data::Tree::ptr prune_tree(const std::string &path,
                             bool do_signal = true);
  
  // property
  bool install_property(GObject *object,
                        const std::string &gobject_property_name,
                        const std::string &name_to_give,
                        const std::string &long_name);
  bool reinstall_property(GObject *replacement_object,
                          const std::string &gobject_property_name,
                          const std::string &name,
                          const std::string &long_name);
  bool install_property_by_pspec(GObject *object,
                                 GParamSpec *pspec,
                                 const std::string &name_to_give,
                                 const std::string &long_name);
  bool uninstall_property(const std::string &name);
  // properties are enabled by default during installation
  bool disable_property(const std::string &name);
  bool enable_property(const std::string &name);

  // methods
  bool install_method(const std::string &long_name,
                      const std::string &method_name,
                      const std::string &short_description,
                      const std::string &return_description,
                      const Method::args_doc &arg_description,
                      Method::method_ptr method,
                      Method::return_type return_type,
                      Method::args_types arg_types,
                      gpointer user_data);
  bool disable_method(const std::string &name);
  bool enable_method(const std::string &name);

  // signals
  bool install_signal(const std::string &long_name,
                      const std::string &signal_name,
                      const std::string &short_description,
                      const Signal::args_doc &arg_description,
                      guint number_of_params, GType *param_types);

  bool install_signal_with_class_name(const std::string &class_name,
                                      const std::string &long_name,
                                      const std::string &signal_name,
                                      const std::string &short_description,
                                      const Signal::args_doc &arg_description,
                                      guint number_of_params,
                                      GType *param_types);
  void signal_emit(std::string signal_name, ...);

  // custom signals
  void emit_on_interface_changed();   // in order to tell properties/methods has changed

  // use a consistent naming for shmdatas
  std::string make_file_name(const std::string &suffix);
  std::string get_quiddity_name_from_file_name(const std::string &shmdata_path);
  std::string get_manager_name();
  
  // used in order to dynamically create other quiddity, weak_ptr is used in order to
  // avoid circular references to the manager_impl
  std::weak_ptr<QuiddityManager_Impl> manager_impl_{};

  // gobject wrapper for custom signals and properties
  GObjectWrapper::ptr gobject_;

  GMainContext *get_g_main_context();
};


#define SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(cpp_quiddity_class,        \
                                             class_name,                \
                                             name,                      \
                                             category,                  \
                                             tags,                      \
                                             description,               \
                                             license,                   \
                                             author)                    \
  QuiddityDocumentation cpp_quiddity_class::switcher_doc_(name,         \
                                                          class_name,   \
                                                          category,     \
                                                          tags,         \
                                                          description,  \
                                                          license,      \
                                                          author);      \
  QuiddityDocumentation *cpp_quiddity_class::get_documentation()        \
  {return &switcher_doc_;}


#define SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(cpp_quiddity_class)    \
  typedef std::shared_ptr<cpp_quiddity_class> ptr;                      \
  QuiddityDocumentation *get_documentation();                           \
  static QuiddityDocumentation switcher_doc_;


#define SWITCHER_DECLARE_PLUGIN(cpp_quiddity_class)             \
  extern "C" Quiddity *create(const std::string &name) {        \
    return new cpp_quiddity_class(name);                        \
  }                                                             \
  extern "C" void destroy(Quiddity *quiddity) {                 \
    delete quiddity;                                            \
  }                                                             \
  extern "C" QuiddityDocumentation *get_documentation() {       \
    return &cpp_quiddity_class::switcher_doc_;                  \
  }

}  // namespace switcher
#endif
