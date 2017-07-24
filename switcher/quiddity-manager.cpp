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

#include <string.h>
#include <fstream>
#include <streambuf>
#include <string>

#include "./gst-utils.hpp"
#include "./information-tree-json.hpp"
#include "./quiddity-manager.hpp"
#include "./quiddity.hpp"
#include "./scope-exit.hpp"

namespace switcher {
QuiddityManager::ptr QuiddityManager::make_manager(const std::string& name) {
  if (!gst_is_initialized()) gst_init(nullptr, nullptr);
  GstRegistry* registry = gst_registry_get();
  // TODO add option for scanning a path
  gst_registry_scan_path(registry, "/usr/local/lib/gstreamer-1.0/");
  gst_registry_scan_path(registry, "/usr/lib/gstreamer-1.0/");

  QuiddityManager::ptr manager(new QuiddityManager(name));
  manager->me_ = manager;
  return manager;
}

QuiddityManager::QuiddityManager(const std::string& name)
    : manager_impl_(QuiddityManager_Impl::make_manager(this, name)), name_(name) {}

std::string QuiddityManager::get_name() const { return name_; }

void QuiddityManager::reset_state(bool remove_created_quiddities) {
  if (remove_created_quiddities) {
    manager_impl_->mute_signal_subscribers(true);
    for (auto& quid : manager_impl_->get_instances()) {
      if (quiddities_at_reset_.cend() ==
          std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid)) {
        manager_impl_->remove(quid);
      }
    }
    manager_impl_->mute_signal_subscribers(false);
  }
  invocations_.clear();
  quiddities_at_reset_ = manager_impl_->get_instances();
}

void QuiddityManager::try_save_current_invocation(const InvocationSpec& invocation_spec) {
  // FIXME do something avoiding this horrible hack:
  std::vector<std::string> excluded = {"connect",
                                       "disconnect",
                                       "last_midi_event_to_property",
                                       "next_midi_event_to_property",
                                       "can-sink-caps",
                                       "send",
                                       "hang-up",
                                       "register",
                                       "unregister",
                                       "add_buddy",
                                       "name_buddy",
                                       "del_buddy",
                                       "set_stun_turn"};

  // save the invocation if required
  if (excluded.end() == std::find(excluded.begin(), excluded.end(), invocation_spec.args_[1]))
    invocations_.push_back(invocation_spec);
}

bool QuiddityManager::load_state(InfoTree::ptr state, bool mute_existing_subscribers) {
  if (!state) return false;
  auto histo_str = std::string("history.");
  auto invocations_paths = state->get_child_keys(histo_str);
  std::vector<InvocationSpec> history;
  for (auto& it : invocations_paths) {
    history.push_back(
        InvocationSpec::get_invocation_spec_from_tree(state->get_tree(histo_str + it)));
  }
  // trees
  auto quiddities_user_data = state->get_tree("userdata.");
  auto quiddities = state->get_tree(".quiddities");
  auto properties = state->get_tree(".properties");
  auto readers = state->get_tree(".readers");
  auto custom_states = state->get_tree(".custom_states");

  bool debug = false;
  if (mute_existing_subscribers) manager_impl_->mute_signal_subscribers(true);
  On_scope_exit {
    if (mute_existing_subscribers) manager_impl_->mute_signal_subscribers(false);
  };

  if (debug) g_print("start playing history\n");

  // making quiddities
  if (quiddities) {
    auto quids = quiddities->get_child_keys(".");
    // creating quiddities
    for (auto& it : quids) {
      std::string quid_class = quiddities->branch_get_value(it);
      if (it != create(quid_class, it)) {
        g_warning("error creating quiddity %s (of type %s)", it.c_str(), quid_class.c_str());
      }
    }
    // loading custom state
    for (auto& it : quids) {
      if (!manager_impl_->has_instance(it)) continue;
      if (custom_states && !custom_states->empty()) {
        manager_impl_->get_quiddity(it)->on_loading(custom_states->get_tree(it));
      } else {
        manager_impl_->get_quiddity(it)->on_loading(InfoTree::make());
      }
    }
  }

  // applying properties
  std::vector<std::string> quid_to_start;
  if (properties) {
    auto quids = properties->get_child_keys(".");
    for (auto& quid : quids) {
      auto props = properties->get_child_keys(quid);
      for (auto& prop : props) {
        if (prop == "started" && properties->branch_get_value(quid + ".started")) {
          quid_to_start.push_back(quid);
        } else {
          if (!use_prop<MPtr(&PContainer::set_str_str)>(
                  quid, prop, Any::to_string(properties->branch_get_value(quid + "." + prop))))
            g_warning("failed to apply value, quiddity is %s, property is %s, value is %s",
                      quid.c_str(),
                      prop.c_str(),
                      Any::to_string(properties->branch_get_value(quid + "." + prop)).c_str());
        }
      }
    }
  }

  invocation_loop_.run([&]() {
    // playing history
    for (auto& it : history) {
      // do not run invocations that not supposed to be saved
      if (debug) {
        g_print("running invocation:");
        for (auto& iter : it.args_) g_print(" %s ", iter.c_str());
        g_print("\n");
        if (!it.vector_arg_.empty()) {
          g_print("            vector args:");
          for (auto& iter : it.vector_arg_) g_print(" %s ", iter.c_str());
          g_print("\n");
        }
      }
      manager_impl_->invoke(it.args_[0], it.args_[1], nullptr, it.vector_arg_);
    }
  });  // invocation_loop_.run

  // applying user data to quiddities
  if (quiddities_user_data) {
    auto quids = quiddities_user_data->get_child_keys(".");
    for (auto& it : quids) {
      if (manager_impl_->has_instance(it)) {
        auto child_keys = quiddities_user_data->get_child_keys(it);
        for (auto& kit : child_keys) {
          manager_impl_->user_data<MPtr(&InfoTree::graft)>(
              it, kit, quiddities_user_data->get_tree(it + "." + kit));
        }
      }
    }
  }
  // starting quiddities
  for (auto& quid : quid_to_start) {
    if (!use_prop<MPtr(&PContainer::set_str_str)>(quid, "started", "true")) {
      g_warning("failed to start quiddity %s", quid.c_str());
    }
  }

  // Connect shmdata
  if (readers) {
    auto quids = readers->get_child_keys(".");
    for (auto& quid : quids) {
      auto tmpreaders = readers->get_child_keys(quid);
      for (auto& reader : tmpreaders) {
        manager_impl_->invoke(quid,
                              "connect",
                              nullptr,
                              {Any::to_string(readers->branch_get_value(quid + "." + reader))});
      }
    }
  }

  // on_loaded
  if (quiddities) {
    auto quids = quiddities->get_child_keys(".");
    for (auto& it : quids) {
      if (!manager_impl_->has_instance(it)) continue;
      manager_impl_->get_quiddity(it)->on_loaded();
    }
  }
  return true;
}

InfoTree::ptr QuiddityManager::get_state() const {
  auto quiddities = manager_impl_->get_instances();
  InfoTree::ptr tree = InfoTree::make();

  // saving history
  for (unsigned int i = 0; i < invocations_.size(); i++) {
    tree->graft(std::string("history.") + std::to_string(i), invocations_[i].get_info_tree());
  }
  tree->tag_as_array("history", true);

  // saving per-quiddity information
  for (auto& quid_name : quiddities) {
    // saving custom tree if some is provided
    auto custom_tree = manager_impl_->get_quiddity(quid_name)->on_saving();
    if (custom_tree && !custom_tree->empty())
      tree->graft(".custom_states." + quid_name, std::move(custom_tree));

    // name and class
    if (quiddities_at_reset_.cend() ==
        std::find(quiddities_at_reset_.cbegin(), quiddities_at_reset_.cend(), quid_name)) {
      auto quid_class =
          manager_impl_->get_quiddity(quid_name)->get_documentation().get_class_name();
      tree->graft(".quiddities." + quid_name, InfoTree::make(quid_class));
    }

    // user data
    auto quid_user_data_tree = user_data<MPtr(&InfoTree::get_tree)>(quid_name, ".");
    if (!quid_user_data_tree->empty()) {
      tree->graft(".userdata." + quid_name, user_data<MPtr(&InfoTree::get_tree)>(quid_name, "."));
    }
    // writable property values
    use_prop<MPtr(&PContainer::update_values_in_tree)>(quid_name);
    auto props = use_tree<MPtr(&InfoTree::get_child_keys)>(quid_name, "property");
    for (auto& prop : props) {
      // Don't save unwritable properties.
      if (!use_tree<MPtr(&InfoTree::branch_get_value)>(
              quid_name, std::string("property.") + prop + ".writable"))
        continue;
      // Don't save properties with saving disabled.
      if (!manager_impl_->get_quiddity(quid_name)->prop_is_saved(prop)) continue;
      tree->graft(".properties." + quid_name + "." + prop,
                  InfoTree::make(use_tree<MPtr(&InfoTree::branch_get_value)>(
                      quid_name, std::string("property.") + prop + ".value")));
    }

    // Record shmdata connections.
    // Ignore them if no connect-to methods is installed for this quiddity.
    if (!manager_impl_->get_quiddity(quid_name)->has_method("connect")) continue;

    auto readers = use_tree<MPtr(&InfoTree::get_child_keys)>(quid_name, "shmdata.reader");
    int nb = 0;
    for (auto& reader : readers) {
      if (!reader.empty()) {
        tree->graft(".readers." + quid_name + ".reader_" + std::to_string(++nb),
                    InfoTree::make(reader));
      }
    }
    tree->tag_as_array(".readers." + quid_name, true);
  }
  // calling on_saved callback
  for (auto& quid_name : quiddities) {
    manager_impl_->get_quiddity(quid_name)->on_saved();
  }

  return tree;
}

bool QuiddityManager::invoke_va(const std::string& quiddity_name,
                                const std::string& method_name,
                                std::string** return_value,
                                ...) {
  std::vector<std::string> method_args;
  va_list vl;
  va_start(vl, return_value);
  char* method_arg = va_arg(vl, char*);
  while (method_arg != nullptr) {
    method_args.push_back(method_arg);
    method_arg = va_arg(vl, char*);
  }
  va_end(vl);

  bool res = false;
  InvocationSpec current_invocation;
  invocation_loop_.run([&]() {
    current_invocation.add_arg(quiddity_name);
    current_invocation.add_arg(method_name);
    current_invocation.set_vector_arg(method_args);

    std::string* result = nullptr;
    if (manager_impl_->invoke(current_invocation.args_[0],
                              current_invocation.args_[1],
                              &result,
                              current_invocation.vector_arg_)) {
      current_invocation.success_ = true;
      if (nullptr == result)
        current_invocation.result_.push_back("error");
      else
        current_invocation.result_.push_back(*result);
    } else
      current_invocation.success_ = false;

    if (nullptr != result) delete result;

    if (return_value != nullptr && !current_invocation.result_.empty())
      *return_value = new std::string(current_invocation.result_[0]);
    res = current_invocation.success_;
    try_save_current_invocation(current_invocation);
  });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::invoke(const std::string& quiddity_name,
                             const std::string& method_name,
                             std::string** return_value,
                             const std::vector<std::string>& args) {
  bool res = false;
  InvocationSpec current_invocation;
  invocation_loop_.run([&]() {
    current_invocation.add_arg(quiddity_name);
    current_invocation.add_arg(method_name);
    current_invocation.set_vector_arg(args);

    std::string* result = nullptr;
    if (manager_impl_->invoke(current_invocation.args_[0],
                              current_invocation.args_[1],
                              &result,
                              current_invocation.vector_arg_)) {
      current_invocation.success_ = true;
      if (nullptr == result)
        current_invocation.result_.push_back("error");
      else
        current_invocation.result_.push_back(*result);
    } else
      current_invocation.success_ = false;

    if (nullptr != result) delete result;

    if (return_value != nullptr && !current_invocation.result_.empty())
      *return_value = new std::string(current_invocation.result_[0]);
    res = current_invocation.success_;
    try_save_current_invocation(current_invocation);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_methods_description(const std::string& quiddity_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_methods_description(quiddity_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_method_description(const std::string& quiddity_name,
                                                    const std::string& method_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_method_description(quiddity_name, method_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_methods_description_by_class(const std::string& class_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_methods_description_by_class(class_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_method_description_by_class(const std::string& class_name,
                                                             const std::string& method_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_method_description_by_class(class_name, method_name);
  });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::has_method(const std::string& quiddity_name, const std::string& method_name) {
  bool res;
  invocation_loop_.run([&]() {
    res = manager_impl_->has_method(quiddity_name, method_name);
  });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::make_signal_subscriber(const std::string& subscriber_name,
                                             QuiddityManager::SignalCallback callback,
                                             void* user_data) {
  bool res;
  invocation_loop_.run([&]() {
    res = manager_impl_->make_signal_subscriber(subscriber_name, callback, user_data);
  });  // invocation_loop_.run

  return res;
}

bool QuiddityManager::remove_signal_subscriber(const std::string& subscriber_name) {
  bool res;
  invocation_loop_.run([&]() {
    res = manager_impl_->remove_signal_subscriber(subscriber_name);
  });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::subscribe_signal(const std::string& subscriber_name,
                                       const std::string& quiddity_name,
                                       const std::string& signal_name) {
  bool res;
  invocation_loop_.run([&]() {
    res = manager_impl_->subscribe_signal(subscriber_name, quiddity_name, signal_name);
  });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::unsubscribe_signal(const std::string& subscriber_name,
                                         const std::string& quiddity_name,
                                         const std::string& signal_name) {
  bool res;
  invocation_loop_.run([&]() {
    res = manager_impl_->unsubscribe_signal(subscriber_name, quiddity_name, signal_name);
  });  // invocation_loop_.run
  return res;
}

std::vector<std::string> QuiddityManager::list_signal_subscribers() {
  std::vector<std::string> res;
  invocation_loop_.run(
      [&]() { res = manager_impl_->list_signal_subscribers(); });  // invocation_loop_.run
  return res;
}

std::vector<std::pair<std::string, std::string>> QuiddityManager::list_subscribed_signals(
    const std::string& subscriber_name) {
  std::vector<std::pair<std::string, std::string>> res;
  invocation_loop_.run([&]() {
    res = manager_impl_->list_subscribed_signals(subscriber_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::list_subscribed_signals_json(const std::string& subscriber_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->list_subscribed_signals_json(subscriber_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_signals_description(const std::string& quiddity_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_signals_description(quiddity_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_signal_description(const std::string& quiddity_name,
                                                    const std::string& signal_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_signal_description(quiddity_name, signal_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_signals_description_by_class(const std::string& class_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_signals_description_by_class(class_name);
  });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_signal_description_by_class(const std::string& class_name,
                                                             const std::string& signal_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_signal_description_by_class(class_name, signal_name);
  });  // invocation_loop_.run
  return res;
}

void QuiddityManager::auto_init(const std::string& quiddity_name) {
  Quiddity::ptr quidd = manager_impl_->get_quiddity(quiddity_name);
  if (!quidd) return;
  QuiddityManagerWrapper::ptr wrapper = std::dynamic_pointer_cast<QuiddityManagerWrapper>(quidd);
  if (wrapper) wrapper->set_quiddity_manager(me_.lock());
}

std::string QuiddityManager::create(const std::string& quiddity_class) {
  std::string res;
  invocation_loop_.run(
      [&]() { res = manager_impl_->create(quiddity_class); });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::scan_directory_for_plugins(const std::string& directory) {
  return manager_impl_->scan_directory_for_plugins(directory);
}

bool QuiddityManager::load_configuration_file(const std::string& file_path) {
  return manager_impl_->load_configuration_file(file_path);
}

std::string QuiddityManager::create(const std::string& quiddity_class,
                                    const std::string& nickname) {
  std::string res;
  invocation_loop_.run(
      [&]() { res = manager_impl_->create(quiddity_class, nickname); });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::remove(const std::string& quiddity_name) {
  bool res;
  invocation_loop_.run(
      [&]() { res = manager_impl_->remove(quiddity_name); });  // invocation_loop_.run
  return res;
}

bool QuiddityManager::has_quiddity(const std::string& name) {
  return manager_impl_->has_instance(name);
}

std::vector<std::string> QuiddityManager::get_classes() {
  std::vector<std::string> res;
  invocation_loop_.run([&]() { res = manager_impl_->get_classes(); });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_classes_doc() {
  std::string res;
  invocation_loop_.run([&]() { res = manager_impl_->get_classes_doc(); });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_class_doc(const std::string& class_name) {
  std::string res;
  invocation_loop_.run(
      [&]() { res = manager_impl_->get_class_doc(class_name); });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_quiddities_description() {
  std::string res;
  invocation_loop_.run(
      [&]() { res = manager_impl_->get_quiddities_description(); });  // invocation_loop_.run
  return res;
}

std::string QuiddityManager::get_quiddity_description(const std::string& quiddity_name) {
  std::string res;
  invocation_loop_.run([&]() {
    res = manager_impl_->get_quiddity_description(quiddity_name);
  });  // invocation_loop_.run
  return res;
}

std::vector<std::string> QuiddityManager::get_quiddities() const {
  std::vector<std::string> res;
  // FIXME this is not thread safe
  res = manager_impl_->get_instances();
  return res;
}

std::string QuiddityManager::get_nickname(const std::string& name) const {
  auto quid = manager_impl_->get_quiddity(name);
  if (!quid) {
    g_debug("quiddity %s not found", name.c_str());
    return std::string();
  }
  return quid->get_nickname();
}

bool QuiddityManager::set_nickname(const std::string& name, const std::string& nickname) {
  auto quid = manager_impl_->get_quiddity(name);
  if (!quid) {
    g_debug("quiddity %s not found", name.c_str());
    return false;
  }
  return quid->set_nickname(nickname);
}

}  // namespace switcher
