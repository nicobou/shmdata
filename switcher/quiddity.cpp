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

#include "./quiddity.hpp"
#include <sys/un.h>
#include <algorithm>
#include <list>
#include <regex>
#include "./gst-utils.hpp"
#include "./information-tree-json.hpp"
#include "./quiddity-container.hpp"
#include "./switcher.hpp"

namespace switcher {

Quiddity::Quiddity(QuiddityConfiguration&& conf)
    : Logged(conf.log_),
      information_tree_(InfoTree::make()),
      structured_user_data_(InfoTree::make()),
      configuration_tree_(conf.tree_config_ ? conf.tree_config_ : InfoTree::make()),
      props_(information_tree_,
             [this](const std::string& key) {
               smanage<MPtr(&SContainer::notify)>(on_tree_grafted_id_, InfoTree::make(key));
             },
             [this](const std::string& key) {
               smanage<MPtr(&SContainer::notify)>(on_tree_pruned_id_, InfoTree::make(key));
             }),
      sigs_(information_tree_,
            [this](const std::string& key) {
              smanage<MPtr(&SContainer::notify)>(on_tree_grafted_id_, InfoTree::make(key));
            },
            [this](const std::string& key) {
              smanage<MPtr(&SContainer::notify)>(on_tree_pruned_id_, InfoTree::make(key));
            }),
      on_method_added_id_(
          smanage<MPtr(&SContainer::make)>("on-method-added", "A method has been installed")),
      on_method_removed_id_(
          smanage<MPtr(&SContainer::make)>("on-method-removed", "A method has been uninstalled")),
      on_tree_grafted_id_(smanage<MPtr(&SContainer::make)>("on-tree-grafted", "On Tree Grafted")),
      on_tree_pruned_id_(smanage<MPtr(&SContainer::make)>(
          "on-tree-pruned", "A tree has been pruned from the quiddity tree")),
      on_user_data_grafted_id_(smanage<MPtr(&SContainer::make)>(
          "on-user-data-grafted", "A tree has been grafted to the quiddity's user data tree")),
      on_user_data_pruned_id_(smanage<MPtr(&SContainer::make)>(
          "on-user-data-pruned", "A branch has been pruned from the quiddity's user data tree")),
      on_nicknamed_id_(smanage<MPtr(&SContainer::make)>(
          "on-nicknamed", "A nickname has been given to the quiddity")),
      methods_description_(std::make_shared<JSONBuilder>()),
      name_(string_to_quiddity_name(conf.name_)),
      nickname_(name_),
      type_(conf.type_),
      qcontainer_(conf.qc_) {
  configuration_tree_->graft(".", InfoTree::make());
  information_tree_->graft(".type", InfoTree::make(conf.type_));
}

Quiddity::~Quiddity() {
  std::lock_guard<std::mutex> lock(self_destruct_mtx_);
}

std::string Quiddity::get_name() const { return name_; }

std::string Quiddity::get_type() const { return type_; }

std::string Quiddity::string_to_quiddity_name(const std::string& name) {
  return std::regex_replace(name, std::regex("[^[:alnum:]| ]"), "-");
}

bool Quiddity::has_method(const std::string& method_name) {
  return (methods_.end() != methods_.find(method_name));
}

bool Quiddity::invoke_method(const std::string& method_name,
                             std::string** return_value,
                             const std::vector<std::string>& args) {
  auto it = methods_.find(method_name);
  if (methods_.end() == it) {
    debug("Quiddity::invoke_method error: method % not found", method_name);
    return false;
  }

  GValue res = G_VALUE_INIT;
  if (false == it->second->invoke(args, &res)) {
    debug("invokation of % failed (missing argments ?)", method_name);
    return false;
  }

  if (return_value != nullptr) {
    gchar* res_val = GstUtils::gvalue_serialize(&res);
    *return_value = new std::string(res_val);
    g_free(res_val);
  }
  g_value_unset(&res);
  return true;
}

bool Quiddity::method_is_registered(const std::string& method_name) {
  return (methods_.end() != methods_.find(method_name) ||
          disabled_methods_.end() != disabled_methods_.find(method_name));
}

bool Quiddity::register_method(const std::string& method_name,
                               Method::method_ptr method,
                               Method::return_type return_type,
                               Method::args_types arg_types,
                               gpointer user_data) {
  if (method == nullptr) {
    debug("fail registering % (method is nullptr)", method_name);
    return false;
  }

  if (method_is_registered(method_name)) {
    debug("registering name % already exists", method_name);
    return false;
  }

  Method::ptr meth(new Method());
  meth->set_method(method, return_type, arg_types, user_data);

  meth->set_position_weight(position_weight_counter_);
  position_weight_counter_ += 20;

  methods_[method_name] = meth;
  return true;
}

bool Quiddity::set_method_description(const std::string& long_name,
                                      const std::string& method_name,
                                      const std::string& short_description,
                                      const std::string& return_description,
                                      const Method::args_doc& arg_description) {
  auto it = methods_.find(method_name);
  if (methods_.end() == it) it = disabled_methods_.find(method_name);

  it->second->set_description(
      long_name, method_name, short_description, return_description, arg_description);
  return true;
}

std::string Quiddity::get_methods_description() {
  methods_description_->reset();
  methods_description_->begin_object();
  methods_description_->set_member_name("methods");
  methods_description_->begin_array();
  std::vector<Method::ptr> methods;
  for (auto& it : methods_) methods.push_back(it.second);
  std::sort(methods.begin(), methods.end(), Categorizable::compare_ptr);
  for (auto& it : methods) methods_description_->add_node_value(it->get_json_root_node());
  methods_description_->end_array();
  methods_description_->end_object();
  return methods_description_->get_string(true);
}

std::string Quiddity::get_method_description(const std::string& method_name) {
  auto it = methods_.find(method_name);
  if (methods_.end() == it) return "{ \"error\" : \" method not found\"}";
  return it->second->get_description();
}

std::string Quiddity::make_file_name(const std::string& suffix) const {
  if (qcontainer_->get_name().empty()) return std::string();
  auto name =
      std::string(get_file_name_prefix() + qcontainer_->get_name() + "_" + name_ + "_" + suffix);

  // Done this way for OSX portability, there is a maximum socket path length in UNIX systems and
  // shmdata use sockets.
  struct sockaddr_un s;
  static auto max_path_size = sizeof(s.sun_path);
  // We need it signed.
  int overflow = static_cast<int>(name.length() - max_path_size);

  // We truncate the quiddity name if it is enough, which should be the case.
  if (overflow > 0) {
    int quiddity_overflow = static_cast<int>(name_.length() - overflow);
    if (quiddity_overflow < 10) {
      name = std::string(get_file_name_prefix() + qcontainer_->get_name() + "_name_error");
      warning(
          "BUG: shmdata name cannot be created properly because it is too long, there is less than "
          "10 characters remaining for the quiddity name, investigate and fix this!");
    } else {
      size_t chunks_size = quiddity_overflow / 2;
      // We split the remaining number of characters in 2 and we take this number at the beginning
      // and the end of the quiddity name.
      auto new_name = std::string(name_.begin(), name_.begin() + chunks_size) +
                      std::string(name_.end() - chunks_size, name_.end());
      name = std::string(get_file_name_prefix() + qcontainer_->get_name() + "_" + new_name + "_" +
                         suffix);
    }
  }

  return name;
}

std::string Quiddity::get_file_name_prefix() const { return "/tmp/switcher_"; }

std::string Quiddity::get_quiddity_name_from_file_name(const std::string& path) const {
  auto file_begin = path.find("switcher_");
  if (std::string::npos == file_begin) {
    warning("%: not a switcher generated path", std::string(__FUNCTION__));
    return std::string();
  }
  std::string filename(path, file_begin);
  // searching for underscores
  std::vector<size_t> underscores;
  bool done = false;
  size_t found = 0;
  while (!done) {
    found = filename.find('_', found);
    if (std::string::npos == found)
      done = true;
    else {
      underscores.push_back(found);
      if (found + 1 == filename.size())
        done = true;
      else
        found = found + 1;
    }
  }
  if (3 != underscores.size()) {
    warning("%: wrong shmdata path format", std::string(__FUNCTION__));
    return std::string();
  }
  // handling bundle: they use there own internal manager named with their actual quiddity name
  auto manager_name =
      std::string(filename, underscores[0] + 1, underscores[1] - (underscores[0] + 1));
  if (qcontainer_->get_name() != manager_name) return manager_name;
  return std::string(filename, underscores[1] + 1, underscores[2] - (underscores[1] + 1));
}

std::string Quiddity::get_shmdata_name_from_file_name(const std::string& path) const {
  size_t pos = 0;
  // Check if external shmdata or regular shmdata.
  if (path.find(get_file_name_prefix()) != std::string::npos) {  // Regular shmdata
    int i = 0;
    while (i < 2) {  // Looking for second '_'
      if (pos != 0) pos += 1;
      pos = path.find('_', pos);
      ++i;
    }
  } else {                  // External shmdata
    pos = path.rfind('/');  // Check last '/' to find the file name only.
  }

  return pos != std::string::npos ? std::string(path, pos + 1) : path;
}

std::string Quiddity::get_manager_name() { return qcontainer_->get_name(); }

std::string Quiddity::get_socket_name_prefix() { return "switcher_"; }

std::string Quiddity::get_socket_dir() { return "/tmp"; }

// methods
bool Quiddity::install_method(const std::string& long_name,
                              const std::string& method_name,
                              const std::string& short_description,
                              const std::string& return_description,
                              const Method::args_doc& arg_description,
                              Method::method_ptr method,
                              Method::return_type return_type,
                              Method::args_types arg_types,
                              gpointer user_data) {
  if (!register_method(method_name, method, return_type, arg_types, user_data)) return false;

  if (!set_method_description(
          long_name, method_name, short_description, return_description, arg_description))
    return false;

  smanage<MPtr(&SContainer::notify)>(on_method_added_id_, InfoTree::make(method_name));
  return true;
}

bool Quiddity::enable_method(const std::string& method_name) {
  auto it = disabled_methods_.find(method_name);
  if (disabled_methods_.end() == it) return false;
  methods_[method_name] = it->second;
  disabled_methods_.erase(it);
  smanage<MPtr(&SContainer::notify)>(on_method_added_id_, InfoTree::make(method_name));
  return true;
}

bool Quiddity::disable_method(const std::string& method_name) {
  auto it = methods_.find(method_name);
  if (methods_.end() == it) return false;
  disabled_methods_[method_name] = it->second;
  methods_.erase(it);
  smanage<MPtr(&SContainer::notify)>(on_method_removed_id_, InfoTree::make(method_name));
  return true;
}

bool Quiddity::graft_tree(const std::string& path, InfoTree::ptr tree, bool do_signal) {
  if (!information_tree_->graft(path, tree)) return false;
  if (do_signal) {
    smanage<MPtr(&SContainer::notify)>(on_tree_grafted_id_, InfoTree::make(path));
  }
  return true;
}

InfoTree::ptr Quiddity::get_tree(const std::string& path) {
  return information_tree_->get_tree(path);
}

InfoTree::ptr Quiddity::prune_tree(const std::string& path, bool do_signal) {
  InfoTree::ptr result = information_tree_->prune(path);
  if (result) {
    if (do_signal) {
      smanage<MPtr(&SContainer::notify)>(on_tree_pruned_id_, InfoTree::make(path));
    }
  } else {
    warning("cannot prune %", path);
  }
  return result;
}

bool Quiddity::user_data_graft_hook(const std::string& path, InfoTree::ptr tree) {
  if (!structured_user_data_->graft(path, std::forward<InfoTree::ptr>(tree))) return false;
  smanage<MPtr(&SContainer::notify)>(on_user_data_grafted_id_, InfoTree::make(path));
  return true;
}

InfoTree::ptr Quiddity::user_data_prune_hook(const std::string& path) {
  auto res = structured_user_data_->prune(path);
  if (res) {
    smanage<MPtr(&SContainer::notify)>(on_user_data_pruned_id_, InfoTree::make(path));
  }
  return res;
}

void Quiddity::self_destruct() {
  std::unique_lock<std::mutex> lock(self_destruct_mtx_);
  auto thread = std::thread([ this, th_lock = std::move(lock) ]() mutable {
    auto self_name = get_name();
    th_lock.unlock();
    if (!qcontainer_->get_switcher()->remove(self_name))
      warning("% did not self destruct", get_name());
  });
  thread.detach();
}

InfoTree::ptr Quiddity::on_saving() { return InfoTree::make(); };

void Quiddity::on_saved(){};

void Quiddity::on_loading(InfoTree::ptr&&){};

void Quiddity::on_loaded(){};

bool Quiddity::prop_is_saved(const std::string& prop) {
  return std::find(properties_blacklist_.begin(), properties_blacklist_.end(), prop) ==
         properties_blacklist_.end();
}

bool Quiddity::toggle_property_saving(const std::string& prop) {
  auto prop_bl = std::find(properties_blacklist_.begin(), properties_blacklist_.end(), prop);
  if (prop_bl == properties_blacklist_.end()) {
    properties_blacklist_.push_back(prop);
    return false;
  } else {
    properties_blacklist_.erase(prop_bl);
    return true;
  }
}

bool Quiddity::set_nickname(const std::string& nickname) {
  nickname_ = nickname;
  smanage<MPtr(&SContainer::notify)>(on_nicknamed_id_, InfoTree::make(nickname));
  return true;
}

std::string Quiddity::get_nickname() const { return nickname_; }

}  // namespace switcher
