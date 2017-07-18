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
    : manager_impl_(QuiddityManager_Impl::make_manager(this, name)),
      name_(name),
      command_(),
      seq_mutex_(),
      command_queue_(g_async_queue_new()),
      invocation_thread_(std::thread(&QuiddityManager::invocation_thread, this)),
      execution_done_cond_(),
      execution_done_mutex_(),
      command_history_() {
  reset_command_history(false);
}

QuiddityManager::~QuiddityManager() {
  clear_command_sync();
  invocation_thread_.join();
}

std::string QuiddityManager::get_name() const { return name_; }

void QuiddityManager::reset_command_history(bool remove_created_quiddities) {
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
  command_history_.history_.clear();
  command_history_.quiddities_user_data_ = InfoTree::make();
  quiddities_at_reset_ = manager_impl_->get_instances();
}

void QuiddityManager::command_lock() {
  seq_mutex_.lock();
  command_.reset(new QuiddityCommand());
}

bool QuiddityManager::must_be_saved(QuiddityCommand::command id) const {
  // actually a white list
  return id == QuiddityCommand::invoke;
}

void QuiddityManager::command_unlock() {
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

  // save the command if required
  if (must_be_saved(command_->id_) &&
      !(excluded.end() != std::find(excluded.begin(), excluded.end(), command_->args_[1]))) {
    command_history_.history_.push_back(command_);
  }
  seq_mutex_.unlock();
}

void QuiddityManager::play_command_history(QuiddityManager::CommandHistory histo,
                                           QuiddityManager::PropCallbackMap* /*prop_cb_data*/,
                                           QuiddityManager::SignalCallbackMap* /*sig_cb_data*/,
                                           bool mute_existing_subscribers) {
  bool debug = false;
  if (mute_existing_subscribers) manager_impl_->mute_signal_subscribers(true);
  On_scope_exit {
    if (mute_existing_subscribers) manager_impl_->mute_signal_subscribers(false);
  };

  if (debug) g_print("start playing history\n");

  // making quiddities
  if (histo.quiddities_) {
    auto quids = histo.quiddities_->get_child_keys(".");
    // creating quiddities
    for (auto& it : quids) {
      std::string quid_class = histo.quiddities_->branch_get_value(it);
      if (it != create(quid_class, it)) {
        g_warning("error creating quiddity %s (of type %s)", it.c_str(), quid_class.c_str());
      }
    }
    // loading custom state
    for (auto& it : quids) {
      if (!manager_impl_->has_instance(it)) continue;
      if (histo.custom_states_ && !histo.custom_states_->empty()) {
        manager_impl_->get_quiddity(it)->on_loading(histo.custom_states_->get_tree(it));
      } else {
        manager_impl_->get_quiddity(it)->on_loading(InfoTree::make());
      }
    }
  }

  // applying properties
  std::vector<std::string> quid_to_start;
  if (histo.properties_) {
    auto quids = histo.properties_->get_child_keys(".");
    for (auto& quid : quids) {
      auto props = histo.properties_->get_child_keys(quid);
      for (auto& prop : props) {
        if (prop == "started" && histo.properties_->branch_get_value(quid + ".started")) {
          quid_to_start.push_back(quid);
        } else {
          if (!use_prop<MPtr(&PContainer::set_str_str)>(
                  quid,
                  prop,
                  Any::to_string(histo.properties_->branch_get_value(quid + "." + prop))))
            g_warning(
                "failed to apply value, quiddity is %s, property is %s, value is %s",
                quid.c_str(),
                prop.c_str(),
                Any::to_string(histo.properties_->branch_get_value(quid + "." + prop)).c_str());
        }
      }
    }
  }

  // playing history
  for (auto& it : histo.history_) {
    // do not run commands that not supposed to be saved
    if (!must_be_saved(it->id_)) continue;
    command_lock();
    command_ = it;
    if (debug) {
      g_print("running command %s args:", QuiddityCommand::get_string_from_id(command_->id_));
      for (auto& iter : command_->args_) g_print(" %s ", iter.c_str());
      g_print("\n");
      if (!command_->vector_arg_.empty()) {
        g_print("            vector args:");
        for (auto& iter : command_->vector_arg_) g_print(" %s ", iter.c_str());
        g_print("\n");
      }
    }
    invoke_in_thread();
    command_unlock();
  }  // end for (auto &it : histo)

  if (debug) g_print("finished playing history\n");
  // applying user data to quiddities
  if (histo.quiddities_user_data_) {
    auto quids = histo.quiddities_user_data_->get_child_keys(".");
    for (auto& it : quids) {
      if (manager_impl_->has_instance(it)) {
        auto child_keys = histo.quiddities_user_data_->get_child_keys(it);
        for (auto& kit : child_keys) {
          manager_impl_->user_data<MPtr(&InfoTree::graft)>(
              it, kit, histo.quiddities_user_data_->get_tree(it + "." + kit));
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
  if (histo.readers_) {
    auto quids = histo.readers_->get_child_keys(".");
    for (auto& quid : quids) {
      auto readers = histo.readers_->get_child_keys(quid);
      for (auto& reader : readers) {
        manager_impl_->invoke(
            quid,
            "connect",
            nullptr,
            {Any::to_string(histo.readers_->branch_get_value(quid + "." + reader))});
      }
    }
  }

  // on_loaded
  if (histo.quiddities_) {
    auto quids = histo.quiddities_->get_child_keys(".");
    for (auto& it : quids) {
      if (!manager_impl_->has_instance(it)) continue;
      manager_impl_->get_quiddity(it)->on_loaded();
    }
  }
}

std::vector<std::string> QuiddityManager::get_signal_subscribers_names(
    const CommandHistory& histo) {
  std::vector<std::string> res;
  for (auto& it : histo.history_)
    if (it->id_ == QuiddityCommand::make_signal_subscriber) res.push_back(it->args_[0]);
  return res;
}

QuiddityManager::CommandHistory QuiddityManager::get_command_history_from_serialization(
    const std::string& save) {
  CommandHistory res;
  // building the tree
  auto tree = JSONSerializer::deserialize(save);
  if (!tree) {
    g_warning("saved history is malformed");
    return res;
  }
  // history
  auto histo_str = std::string("history.");
  auto commands_paths = tree->get_child_keys(histo_str);
  for (auto& it : commands_paths) {
    res.history_.push_back(QuiddityCommand::make_command_from_tree(tree->get_tree(histo_str + it)));
  }
  // trees
  res.quiddities_user_data_ = tree->get_tree("userdata.");
  res.quiddities_ = tree->get_tree(".quiddities");
  res.properties_ = tree->get_tree(".properties");
  res.readers_ = tree->get_tree(".readers");
  res.custom_states_ = tree->get_tree(".custom_states");
  return res;
}

QuiddityManager::CommandHistory QuiddityManager::get_command_history_from_file(
    const char* file_path) {
  CommandHistory res;
  // opening file
  std::ifstream file_stream(file_path);
  if (!file_stream) {
    g_warning("cannot open %s for loading history", file_path);
    return res;
  }
  // get file content into a string
  file_stream.seekg(0, std::ios::end);
  auto size = file_stream.tellg();
  if (0 == size) {
    g_warning("file %s is empty", file_path);
    return res;
  }
  std::string file_str;
  file_str.reserve(size);
  file_stream.seekg(0, std::ios::beg);
  file_str.assign((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());
  return get_command_history_from_serialization(file_str);
}

std::string QuiddityManager::get_serialized_command_history() const {
  auto quiddities = manager_impl_->get_instances();
  InfoTree::ptr tree = InfoTree::make();

  // saving history
  for (unsigned int i = 0; i < command_history_.history_.size(); i++) {
    tree->graft(std::string("history.") + std::to_string(i),
                command_history_.history_[i]->get_info_tree());
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
          manager_impl_->get_quiddity(quid_name)->get_documentation()->get_class_name();
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

  return JSONSerializer::serialize(tree.get());
}

bool QuiddityManager::save_command_history(const char* file_path) const {
  GFile* file = g_file_new_for_commandline_arg(file_path);
  On_scope_exit { g_object_unref(file); };
  GError* error = nullptr;
  GFileOutputStream* file_stream = g_file_replace(file,
                                                  nullptr,
                                                  TRUE,  // make backup
                                                  G_FILE_CREATE_NONE,
                                                  nullptr,
                                                  &error);
  On_scope_exit { g_object_unref(file_stream); };
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return false;
  }

  auto histo = get_serialized_command_history();
  if (histo.empty()) return false;

  // saving the save tree to the file
  gchar* history = g_strdup(histo.c_str());
  g_output_stream_write(
      (GOutputStream*)file_stream, history, sizeof(gchar) * strlen(history), nullptr, &error);
  g_free(history);
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return false;
  }

  g_output_stream_close((GOutputStream*)file_stream, nullptr, &error);
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return false;
  }

  return true;
}

bool QuiddityManager::invoke_va(const std::string& quiddity_name,
                                const std::string& method_name,
                                std::string** return_value,
                                ...) {
  command_lock();
  std::vector<std::string> method_args;
  command_->set_id(QuiddityCommand::invoke);
  if (quiddity_name.empty()) {
    g_warning("trying to invoke with a quiddity with empty name");
    command_unlock();
    return false;
  }
  if (method_name.empty()) {
    g_warning("trying to invoke with a method with empty name");
    command_unlock();
    return false;
  }

  va_list vl;
  va_start(vl, return_value);
  char* method_arg = va_arg(vl, char*);
  while (method_arg != nullptr) {
    method_args.push_back(method_arg);
    method_arg = va_arg(vl, char*);
  }
  va_end(vl);

  command_->add_arg(quiddity_name);
  command_->add_arg(method_name);
  command_->set_vector_arg(method_args);
  invoke_in_thread();
  if (return_value != nullptr && !command_->result_.empty())
    *return_value = new std::string(command_->result_[0]);
  bool res = command_->success_;
  command_unlock();
  return res;
}

bool QuiddityManager::invoke(const std::string& quiddity_name,
                             const std::string& method_name,
                             std::string** return_value,
                             const std::vector<std::string>& args) {
  // std::string res;
  command_lock();
  command_->set_id(QuiddityCommand::invoke);
  command_->add_arg(quiddity_name);
  command_->add_arg(method_name);
  command_->set_vector_arg(args);
  invoke_in_thread();
  if (return_value != nullptr && !command_->result_.empty())
    *return_value = new std::string(command_->result_[0]);
  bool res = command_->success_;
  command_unlock();
  return res;
}

std::string QuiddityManager::get_methods_description(const std::string& quiddity_name) {
  return seq_invoke(QuiddityCommand::get_methods_description, quiddity_name.c_str(), nullptr);
}

std::string QuiddityManager::get_method_description(const std::string& quiddity_name,
                                                    const std::string& method_name) {
  return seq_invoke(
      QuiddityCommand::get_method_description, quiddity_name.c_str(), method_name.c_str(), nullptr);
}

std::string QuiddityManager::get_methods_description_by_class(const std::string& class_name) {
  return seq_invoke(QuiddityCommand::get_methods_description_by_class, class_name.c_str(), nullptr);
}

std::string QuiddityManager::get_method_description_by_class(const std::string& class_name,
                                                             const std::string& method_name) {
  return seq_invoke(QuiddityCommand::get_method_description_by_class,
                    class_name.c_str(),
                    method_name.c_str(),
                    nullptr);
}

bool QuiddityManager::has_method(const std::string& quiddity_name, const std::string& method_name) {
  // FIXME do not have this
  command_lock();
  command_->set_id(QuiddityCommand::has_method);
  command_->add_arg(quiddity_name);
  command_->add_arg(method_name);
  bool res = manager_impl_->has_method(quiddity_name, method_name);
  if (res)
    command_->result_.push_back("true");
  else
    command_->result_.push_back("false");
  command_unlock();
  return res;
}

// bool
// QuiddityManager::has_property(const std::string &quiddity_name,
//                               const std::string &property_name) {
//   return manager_impl_->has_property(quiddity_name, property_name);
// }

bool QuiddityManager::make_signal_subscriber(const std::string& subscriber_name,
                                             QuiddityManager::SignalCallback callback,
                                             void* user_data) {
  command_lock();
  command_->set_id(QuiddityCommand::make_signal_subscriber);
  command_->add_arg(subscriber_name);
  bool res = manager_impl_->make_signal_subscriber(subscriber_name, callback, user_data);
  if (res)
    command_->result_.push_back("true");
  else
    command_->result_.push_back("false");
  command_unlock();

  return res;
}

bool QuiddityManager::remove_signal_subscriber(const std::string& subscriber_name) {
  command_lock();
  command_->set_id(QuiddityCommand::remove_signal_subscriber);
  command_->add_arg(subscriber_name);
  bool res = manager_impl_->remove_signal_subscriber(subscriber_name);
  if (res)
    command_->result_.push_back("true");
  else
    command_->result_.push_back("false");
  command_unlock();
  return res;
}

bool QuiddityManager::subscribe_signal(const std::string& subscriber_name,
                                       const std::string& quiddity_name,
                                       const std::string& signal_name) {
  std::string res = seq_invoke(QuiddityCommand::subscribe_signal,
                               subscriber_name.c_str(),
                               quiddity_name.c_str(),
                               signal_name.c_str(),
                               nullptr);
  return res == "true";
}

bool QuiddityManager::unsubscribe_signal(const std::string& subscriber_name,
                                         const std::string& quiddity_name,
                                         const std::string& signal_name) {
  std::string res = seq_invoke(QuiddityCommand::unsubscribe_signal,
                               subscriber_name.c_str(),
                               quiddity_name.c_str(),
                               signal_name.c_str(),
                               nullptr);
  return res == "true";
  // command_lock ();
  // command_->set_id (QuiddityCommand::unsubscribe_signal);
  // command_->add_arg (subscriber_name);
  // command_->add_arg (quiddity_name);
  // command_->add_arg (signal_name);

  // bool res = manager_impl_->unsubscribe_signal (subscriber_name,
  // quiddity_name, signal_name);
  //  if (res)
  //    command_->result_.push_back("true");
  //  else
  //    command_->result_.push_back("false");

  // command_unlock ();
  // return res;
}

std::vector<std::string> QuiddityManager::list_signal_subscribers() {
  command_lock();
  command_->set_id(QuiddityCommand::list_signal_subscribers);
  std::vector<std::string> res = manager_impl_->list_signal_subscribers();
  command_->result_ = res;
  command_unlock();
  return res;
}

std::vector<std::pair<std::string, std::string>> QuiddityManager::list_subscribed_signals(
    const std::string& subscriber_name) {
  command_lock();
  command_->set_id(QuiddityCommand::list_subscribed_signals);
  std::vector<std::pair<std::string, std::string>> res =
      manager_impl_->list_subscribed_signals(subscriber_name);
  // FIXME no result...
  command_unlock();
  return res;
}

std::string QuiddityManager::list_subscribed_signals_json(const std::string& subscriber_name) {
  command_lock();
  command_->set_id(QuiddityCommand::list_subscribed_signals_json);
  command_->add_arg(subscriber_name);
  std::string res = manager_impl_->list_subscribed_signals_json(subscriber_name);
  command_->result_.push_back(res);
  command_unlock();
  return res;
}

std::string QuiddityManager::get_signals_description(const std::string& quiddity_name) {
  command_lock();
  command_->set_id(QuiddityCommand::get_signals_description);
  command_->add_arg(quiddity_name);
  std::string res = manager_impl_->get_signals_description(quiddity_name);
  command_->result_.push_back(res);
  command_unlock();
  return res;
}

std::string QuiddityManager::get_signal_description(const std::string& quiddity_name,
                                                    const std::string& signal_name) {
  command_lock();
  command_->set_id(QuiddityCommand::get_signal_description);
  command_->add_arg(quiddity_name);
  command_->add_arg(signal_name);
  std::string res = manager_impl_->get_signal_description(quiddity_name, signal_name);
  command_->result_.push_back(res);
  command_unlock();
  return res;
}

std::string QuiddityManager::get_signals_description_by_class(const std::string& class_name) {
  command_lock();
  command_->set_id(QuiddityCommand::get_signals_description_by_class);
  command_->add_arg(class_name);
  std::string res = manager_impl_->get_signals_description_by_class(class_name);
  command_->result_.push_back(res);
  command_unlock();
  return res;
}

std::string QuiddityManager::get_signal_description_by_class(const std::string& class_name,
                                                             const std::string& signal_name) {
  command_lock();
  command_->set_id(QuiddityCommand::get_signal_description_by_class);
  command_->add_arg(class_name);
  command_->add_arg(signal_name);
  std::string res = manager_impl_->get_signal_description_by_class(class_name, signal_name);
  command_->result_.push_back(res);
  command_unlock();
  return res;
}

void QuiddityManager::auto_init(const std::string& quiddity_name) {
  Quiddity::ptr quidd = manager_impl_->get_quiddity(quiddity_name);
  if (!quidd) return;
  QuiddityManagerWrapper::ptr wrapper = std::dynamic_pointer_cast<QuiddityManagerWrapper>(quidd);
  if (wrapper) wrapper->set_quiddity_manager(me_.lock());
}

std::string QuiddityManager::create(const std::string& quiddity_class) {
  std::string res = seq_invoke(QuiddityCommand::create, quiddity_class.c_str(), nullptr);
  return res;
}

bool QuiddityManager::scan_directory_for_plugins(const std::string& directory) {
  return manager_impl_->scan_directory_for_plugins(directory);
}

bool QuiddityManager::load_configuration_file(const std::string& file_path) {
  return manager_impl_->load_configuration_file(file_path);
}

std::string QuiddityManager::create(const std::string& quiddity_class,
                                    const std::string& nick_name) {
  std::string res = seq_invoke(
      QuiddityCommand::create_nick_named, quiddity_class.c_str(), nick_name.c_str(), nullptr);
  return res;
}

bool QuiddityManager::remove(const std::string& quiddity_name) {
  std::string res = seq_invoke(QuiddityCommand::remove, quiddity_name.c_str(), nullptr);
  if (res == "true")
    return true;
  else
    return false;
}

bool QuiddityManager::has_quiddity(const std::string& name) {
  return manager_impl_->has_instance(name);
}

std::vector<std::string> QuiddityManager::get_classes() {
  std::vector<std::string> res;
  command_lock();
  command_->set_id(QuiddityCommand::get_classes);
  invoke_in_thread();
  res = command_->result_;
  command_unlock();
  return res;
}

std::string QuiddityManager::get_classes_doc() {
  return seq_invoke(QuiddityCommand::get_classes_doc, nullptr);
}

std::string QuiddityManager::get_class_doc(const std::string& class_name) {
  return seq_invoke(QuiddityCommand::get_class_doc, class_name.c_str(), nullptr);
}

std::string QuiddityManager::get_quiddities_description() {
  return seq_invoke(QuiddityCommand::get_quiddities_description, nullptr);
}

std::string QuiddityManager::get_quiddity_description(const std::string& quiddity_name) {
  return seq_invoke(QuiddityCommand::get_quiddity_description, quiddity_name.c_str(), nullptr);
}

std::vector<std::string> QuiddityManager::get_quiddities() const {
  std::vector<std::string> res;
  std::lock_guard<std::mutex> lock(seq_mutex_);
  res = manager_impl_->get_instances();
  return res;
}

void QuiddityManager::invocation_thread() {
  bool loop = true;
  while (loop) {
    g_async_queue_pop(command_queue_);
    if (command_->id_ != QuiddityCommand::quit)
      execute_command();
    else
      loop = false;
    {
      std::unique_lock<std::mutex> lock(execution_done_mutex_);
      execution_done_cond_.notify_all();
    }
  }
}

void QuiddityManager::clear_command_sync() {
  command_lock();
  command_->set_id(QuiddityCommand::quit);
  invoke_in_thread();
  g_async_queue_unref(command_queue_);
  command_unlock();
}

// works for char *args only. Use nullptr sentinel
std::string QuiddityManager::seq_invoke(QuiddityCommand::command command, ...) {
  std::string res;
  command_lock();
  va_list vl;
  va_start(vl, command);
  command_->set_id(command);
  char* command_arg = va_arg(vl, char*);
  while (command_arg != nullptr) {
    command_->add_arg(command_arg);
    command_arg = va_arg(vl, char*);
  }
  va_end(vl);

  invoke_in_thread();
  res = command_->result_[0];
  if (command_->id_ == QuiddityCommand::create ||
      command_->id_ == QuiddityCommand::create_nick_named)
    auto_init(command_->result_[0]);
  command_unlock();

  return res;
}

void QuiddityManager::invoke_in_thread() {
  std::unique_lock<std::mutex> lock(execution_done_mutex_);
  g_async_queue_push(command_queue_, command_.get());
  execution_done_cond_.wait(lock);
}

void QuiddityManager::execute_command() {
  switch (command_->id_) {
    case QuiddityCommand::set_property:
      if (manager_impl_->props<MPtr(&PContainer::set_str_str)>(
              command_->args_[0], command_->args_[1], command_->args_[2]))
        command_->result_.push_back("true");
      else
        command_->result_.push_back("false");
      break;
    case QuiddityCommand::get_classes:
      command_->result_ = manager_impl_->get_classes();
      break;
    case QuiddityCommand::get_classes_doc:
      command_->result_.push_back(manager_impl_->get_classes_doc());
      break;
    case QuiddityCommand::get_class_doc:
      command_->result_.push_back(manager_impl_->get_class_doc(command_->args_[0]));
      break;
    case QuiddityCommand::get_quiddities_description:
      command_->result_.push_back(manager_impl_->get_quiddities_description());
      break;
    case QuiddityCommand::get_quiddity_description:
      command_->result_.push_back(manager_impl_->get_quiddity_description(command_->args_[0]));
      break;
    case QuiddityCommand::create:
      command_->result_.push_back(manager_impl_->create(command_->args_[0]));
      break;
    case QuiddityCommand::create_nick_named:
      command_->result_.push_back(manager_impl_->create(command_->args_[0], command_->args_[1]));
      break;
    case QuiddityCommand::remove:
      if (manager_impl_->remove(command_->args_[0]))
        command_->result_.push_back("true");
      else
        command_->result_.push_back("false");
      break;
    case QuiddityCommand::get_methods_description:
      command_->result_.push_back(manager_impl_->get_methods_description(command_->args_[0]));
      break;
    case QuiddityCommand::get_method_description:
      command_->result_.push_back(
          manager_impl_->get_method_description(command_->args_[0], command_->args_[1]));
      break;
    case QuiddityCommand::get_methods_description_by_class:
      command_->result_.push_back(
          manager_impl_->get_methods_description_by_class(command_->args_[0]));
      break;
    case QuiddityCommand::get_method_description_by_class:
      command_->result_.push_back(
          manager_impl_->get_method_description_by_class(command_->args_[0], command_->args_[1]));
      break;
    case QuiddityCommand::invoke: {
      std::string* result = nullptr;
      if (manager_impl_->invoke(
              command_->args_[0], command_->args_[1], &result, command_->vector_arg_)) {
        command_->success_ = true;  // result_.push_back ("true");
        if (nullptr == result)
          command_->result_.push_back("error");
        else
          command_->result_.push_back(*result);
      } else
        command_->success_ = false;  // result_.push_back ("false");

      if (nullptr != result) delete result;
    } break;
    case QuiddityCommand::subscribe_signal:
      if (manager_impl_->subscribe_signal(
              command_->args_[0], command_->args_[1], command_->args_[2]))
        command_->result_.push_back("true");
      else
        command_->result_.push_back("false");
      break;
    case QuiddityCommand::unsubscribe_signal:
      if (manager_impl_->unsubscribe_signal(
              command_->args_[0], command_->args_[1], command_->args_[2]))
        command_->result_.push_back("true");
      else
        command_->result_.push_back("false");
      break;
    default:
      g_debug("** unknown command, cannot launch %s\n",
              QuiddityCommand::get_string_from_id(command_->id_));
  }
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
