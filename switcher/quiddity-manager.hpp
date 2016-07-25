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
 * The QuiddityManager class
 */

#ifndef __SWITCHER_QUIDDITY_MANAGER_H__
#define __SWITCHER_QUIDDITY_MANAGER_H__

#include <stdarg.h>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "./information-tree.hpp"
#include "./make-consultable.hpp"
#include "./quiddity-command.hpp"
#include "./quiddity-manager-impl.hpp"
#include "./quiddity-manager-wrapper.hpp"

namespace switcher {
class QuiddityManager {
 public:
  using ptr = std::shared_ptr<QuiddityManager>;
  struct CommandHistory {
    std::vector<QuiddityCommand::ptr> history_{};
    InfoTree::ptr quiddities_user_data_{nullptr};
    InfoTree::ptr quiddities_{nullptr};
    InfoTree::ptr properties_{nullptr};
    bool empty() { return history_.empty() && !quiddities_; }
  };
  using PropCallback = std::function<void(const std::string& val)>;
  using SignalCallback = void (*)(const std::string& subscriber_name,
                                  const std::string& quiddity_name,
                                  const std::string& signal_name,
                                  const std::vector<std::string>& params,
                                  void* user_data);
  using PropCallbackMap = std::map<std::string, std::pair<PropCallback, void*>>;
  using SignalCallbackMap = std::map<std::string, std::pair<SignalCallback, void*>>;

  ~QuiddityManager();
  static QuiddityManager::ptr make_manager(const std::string& name);
  QuiddityManager& operator=(const QuiddityManager&) = delete;
  QuiddityManager(const QuiddityManager&) = delete;
  std::string get_name() const;

  // *************** command history
  // ***********************************************************
  bool save_command_history(const char* file_path) const;
  static CommandHistory get_command_history_from_file(const char* file_path);
  std::vector<std::string> get_signal_subscribers_names(const CommandHistory& histo);
  void play_command_history(QuiddityManager::CommandHistory histo,
                            QuiddityManager::PropCallbackMap* prop_cb_data,
                            QuiddityManager::SignalCallbackMap* sig_cb_data,
                            bool mute_signal_subscribers);
  void reset_command_history(bool remove_created_quiddities);

  // ************** plugins
  // *******************************************************************
  bool scan_directory_for_plugins(const std::string& directory);

  // ************** configuration
  // *******************************************************************
  bool load_configuration_file(const std::string& file_path);

  // ***************** inspect
  // ****************************************************************
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
  bool has_quiddity(const std::string& name);

  // ****************** informations ******

  Forward_consultable(
      QuiddityManager, QuiddityManager_Impl, manager_impl_.get(), use_tree, use_tree);

  Forward_delegate(
      QuiddityManager, QuiddityManager_Impl, manager_impl_.get(), user_data, user_data);

  // ****************** properties ********
  Forward_consultable(QuiddityManager, QuiddityManager_Impl, manager_impl_.get(), props, use_prop);

  // *********************** methods
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

  // ************************ signals
  // doc (json formatted)
  std::string get_signals_description(const std::string& quiddity_name);
  std::string get_signal_description(const std::string& quiddity_name,
                                     const std::string& signal_name);
  // following "by_class" methods provide properties available after creation
  // only,
  // avoiding possible properties created dynamically
  std::string get_signals_description_by_class(const std::string& class_name);
  std::string get_signal_description_by_class(const std::string& class_name,
                                              const std::string& signal_name);

  bool make_signal_subscriber(const std::string& subscriber_name,
                              /* void (*callback)(std::string subscriber_name, */
                              /*     std::string quiddity_name, */
                              /*     std::string signal_name, */
                              /*     std::vector<std::string> params, */
                              /*     void *user_data) */
                              QuiddityManager::SignalCallback callback,
                              void* user_data);
  bool remove_signal_subscriber(const std::string& subscriber_name);
  bool subscribe_signal(const std::string& subscriber_name,
                        const std::string& quiddity_name,
                        const std::string& signal_name);
  bool unsubscribe_signal(const std::string& subscriber_name,
                          const std::string& quiddity_name,
                          const std::string& signal_name);

  std::vector<std::string> list_signal_subscribers();
  std::vector<std::pair<std::string, std::string>> list_subscribed_signals(
      const std::string& subscriber_name);
  // json
  std::string list_subscribed_signals_json(const std::string& subscriber_name);

 private:
  QuiddityManager_Impl::ptr manager_impl_;  // may be shared with others for
                                            // automatic quiddity creation
  std::string name_;
  // running commands in sequence:
  mutable QuiddityCommand::ptr command_;  // current command
  mutable std::mutex seq_mutex_;
  GAsyncQueue* command_queue_;
  std::vector<std::string> quiddities_at_reset_{};
  // invocation of commands in a dedicated thread
  std::thread invocation_thread_;
  std::condition_variable execution_done_cond_;
  std::mutex execution_done_mutex_;
  // gives shared pointer to this:
  std::weak_ptr<QuiddityManager> me_{};
  // history
  mutable CommandHistory command_history_;
  gint64 history_begin_time_;  // monotonic time, in microseconds

  QuiddityManager() = delete;
  explicit QuiddityManager(const std::string& name);
  void auto_init(const std::string& quiddity_name);
  void command_lock();
  void command_unlock();
  std::string seq_invoke(QuiddityCommand::command command, ...);
  void clear_command_sync();
  void invocation_thread();
  void execute_command();
  void invoke_in_thread();
  bool must_be_saved(QuiddityCommand::command id) const;
};
}  // namespace switcher

#endif
