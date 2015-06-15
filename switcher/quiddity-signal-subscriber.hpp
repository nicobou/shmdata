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
 * The Quiddity signal subscriber
 */

#ifndef __SWITCHER_QUIDDITY_SIGNAL_SUBSCRIBER_H__
#define __SWITCHER_QUIDDITY_SIGNAL_SUBSCRIBER_H__

#include <string>
#include <memory>
#include <map>
#include <vector>
#include "./signal-string.hpp"

namespace switcher {
class Quiddity;
class QuiddityManager_Impl;

class QuidditySignalSubscriber {
 public:
  typedef std::shared_ptr<QuidditySignalSubscriber> ptr;
  typedef void (*OnEmittedCallback) (const std::string &subscriber_name,
                                     const std::string &quiddity_name,
                                     const std::string &signal_name,
                                     const std::vector<std::string> &params,
                                     void *user_data);
  QuidditySignalSubscriber();
  ~QuidditySignalSubscriber();
  QuidditySignalSubscriber(const QuidditySignalSubscriber &) = delete;
  QuidditySignalSubscriber &operator=(const QuidditySignalSubscriber &)
  = delete;
  void mute(bool muted);

  void set_callback(OnEmittedCallback cb);
  void set_user_data(void *user_data);
  void set_name(const gchar *name);
  bool subscribe(std::shared_ptr<Quiddity> quid,
                 const std::string &signal_name);
  bool unsubscribe(std::shared_ptr<Quiddity> quid,
                   const std::string &signal_name);
  bool unsubscribe(std::shared_ptr<Quiddity> quid);

  std::vector<std::pair<std::string, std::string>> list_subscribed_signals();
  static void signal_cb(std::vector<std::string> params,
                        gpointer user_data);

  // manager_impl initialization
  void set_manager_impl(std::shared_ptr<QuiddityManager_Impl>
                        manager_impl);

 private:
  bool muted_{false};
  OnEmittedCallback user_callback_{nullptr};
  void *user_data_{nullptr};
  std::string name_{};
  std::weak_ptr<QuiddityManager_Impl> manager_impl_{};

  typedef struct {
    QuidditySignalSubscriber *subscriber{nullptr};
    std::string name{};
    std::string quiddity_name{};
    std::string signal_name{};
    OnEmittedCallback user_callback{nullptr};
    void *user_data{nullptr};
    std::weak_ptr<Quiddity> quid{};
  } SignalData;
  typedef std::map<std::pair<std::string, std::string>,
                   SignalData *>SignalDataMap;
  SignalDataMap signal_datas_{};
};
}  // namespace switcher

#endif
