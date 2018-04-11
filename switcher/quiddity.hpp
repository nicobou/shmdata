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

#include <string.h>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "./documentation-registry.hpp"
#include "./information-tree.hpp"
#include "./json-builder.hpp"
#include "./logged.hpp"
#include "./make-consultable.hpp"
#include "./method-container.hpp"
#include "./method.hpp"
#include "./property-container.hpp"
#include "./quiddity-configuration.hpp"
#include "./quiddity-documentation.hpp"
#include "./safe-bool-idiom.hpp"
#include "./signal-container.hpp"

namespace switcher {
class Container;

class Quiddity : public Logged, public SafeBoolIdiom {
  friend class Bundle;  // access to props_ in order to forward properties
  // FIXME do something for this (to many friend class in quiddity.hpp):
  friend class ProtocolCurl;
  friend class ProtocolOsc;
  friend class ProtocolReader;
  friend class StartableQuiddity;
  friend class ShmdataConnector;
  friend class ShmdataWriter;
  friend class ShmdataFollower;
  friend class GstVideoCodec;
  friend class GstAudioCodec;
  friend class ShmdataDecoder;
  friend struct ShmdataStat;

 public:
  using ptr = std::shared_ptr<Quiddity>;
  using wptr = std::weak_ptr<Quiddity>;
  explicit Quiddity(quid::Config&&);
  Quiddity() = delete;
  Quiddity(const Quiddity&) = delete;
  Quiddity& operator=(const Quiddity&) = delete;
  virtual ~Quiddity();

  // save/load quiddity state
  virtual InfoTree::ptr on_saving();
  virtual void on_saved();
  virtual void on_loading(InfoTree::ptr&& tree);
  virtual void on_loaded();
  bool prop_is_saved(const std::string& prop);

  // instance name
  std::string get_name() const;
  std::string get_type() const;
  static std::string string_to_quiddity_name(const std::string& name);
  bool set_nickname(const std::string& nickname);
  std::string get_nickname() const;

  // properties
  Make_consultable(Quiddity, PContainer, &props_, prop);

  // methods
  Make_consultable(Quiddity, MContainer, &meths_, meth);

  // signals
  Make_consultable(Quiddity, SContainer, &sigs_, sig);

  // information
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

  // use a consistent naming for shmdatas
  std::string make_shmpath(const std::string& suffix) const;
  std::string get_manager_name();
  std::string get_quiddity_name_from_file_name(const std::string& shmdata_path) const;
  std::string get_shmdata_name_from_file_name(const std::string& path) const;
  std::string get_shmpath_prefix() const;

 private:
  // safe bool idiom implementation
  bool safe_bool_idiom() const { return is_valid_; }
  // user data hooks
  bool user_data_graft_hook(const std::string& path, InfoTree::ptr tree);
  InfoTree::ptr user_data_prune_hook(const std::string& path);

  // tree used by quiddity to communicate info to user,
  // read-only by user, read/write by quiddity
  InfoTree::ptr information_tree_;

  // writable tree for custom user data, should not be used by quiddity
  // (hooks are installed for signaling graft and prune)
  InfoTree::ptr structured_user_data_;

  // configuration tree. When manager is loaded with a config file,
  // the branch of the tree corresponding to the quiddity type
  // is given to the quiddity. There is no constrain about how quiddity
  // should use this configuration
  InfoTree::ptr configuration_tree_;

  // signals
  SContainer sigs_;
  SContainer::sig_id_t on_method_added_id_;
  SContainer::sig_id_t on_method_removed_id_;
  SContainer::sig_id_t on_tree_grafted_id_;
  SContainer::sig_id_t on_tree_pruned_id_;
  SContainer::sig_id_t on_user_data_grafted_id_;
  SContainer::sig_id_t on_user_data_pruned_id_;
  SContainer::sig_id_t on_nicknamed_id_;

  // properties
  PContainer props_;
  std::vector<std::string> properties_blacklist_{};

  // methods
  MContainer meths_;

  // position weight FIXME should be outside this file ?
  gint position_weight_counter_{0};

  // naming
  std::string name_;
  std::string nickname_;
  std::string type_;

  // life management
  std::mutex self_destruct_mtx_{};


 protected:
  // information
  bool graft_tree(const std::string& path, InfoTree::ptr tree_to_graft, bool do_signal = true);
  InfoTree::ptr prune_tree(const std::string& path, bool do_signal = true);
  InfoTree::ptr get_tree(const std::string& path);

  // property
  Make_delegate(Quiddity, PContainer, &props_, pmanage);

  // configuration
  Make_consultable(Quiddity, InfoTree, configuration_tree_.get(), config);

  // signal
  Make_delegate(Quiddity, SContainer, &sigs_, smanage);

  // methods
  Make_delegate(Quiddity, MContainer, &meths_, mmanage);

  // life management
  void self_destruct();

  bool toggle_property_saving(const std::string&);

  // safe bool idiom implementation, default to valid
  // if you are writing a quiddity, you need to set this to false in order to invalidate the
  // creation of the Quiddity instance
  bool is_valid_{true};

  // access to the quiddity Container
  quid::Container* qcontainer_;
};

#define SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(                                                 \
    cpp_quiddity_class, class_name, name, category, tags, description, license, author)       \
  bool cpp_quiddity_class##_doc_registered = DocumentationRegistry::get()->register_doc(      \
      class_name, quid::Doc(class_name, name, category, tags, description, license, author)); \
  bool cpp_quiddity_class##_class_registered =                                                \
      DocumentationRegistry::get()->register_type_from_class_name(                            \
          std::string(#cpp_quiddity_class), class_name);

#define SWITCHER_DECLARE_PLUGIN(cpp_quiddity_class)                         \
  extern "C" Quiddity* create(quid::Config&& conf) {                        \
    return new cpp_quiddity_class(std::forward<quid::Config>(conf));        \
  }                                                                         \
  extern "C" void destroy(Quiddity* quiddity) { delete quiddity; }          \
  extern "C" const char* get_quiddity_type() {                              \
    static char type[64];                                                   \
    strcpy(type,                                                            \
           DocumentationRegistry::get()                                     \
               ->get_type_from_class_name(std::string(#cpp_quiddity_class)) \
               .c_str());                                                   \
    return static_cast<const char*>(type);                                  \
  }
}  // namespace switcher
#endif
