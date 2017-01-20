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

#include "./quiddity-signal-subscriber.hpp"
#include "./quiddity-manager-impl.hpp"
#include "./quiddity.hpp"

namespace switcher {
QuidditySignalSubscriber::QuidditySignalSubscriber() {}

QuidditySignalSubscriber::~QuidditySignalSubscriber() {
  for (auto& it : signal_datas_) {
    Quiddity::ptr quid = it.second->quid.lock();
    if ((bool)quid) {
      g_debug(
          "QuidditySignalSubscriber: cleaning signal not unsubscribed %s, %s, "
          "%s",
          it.second->name.c_str(),
          it.second->quiddity_name.c_str(),
          it.second->signal_name.c_str());
      quid->unsubscribe_signal(it.second->signal_name, signal_cb, it.second);
    }
    delete (it.second);
  }
}

void QuidditySignalSubscriber::mute(bool muted) { muted_ = muted; }

void QuidditySignalSubscriber::signal_cb(const std::vector<std::string>& params,
                                         gpointer user_data) {
  SignalData* signal = static_cast<SignalData*>(user_data);
  if (!signal->subscriber->muted_)
    signal->user_callback(signal->name.c_str(),
                          signal->quiddity_name.c_str(),
                          signal->signal_name.c_str(),
                          params,
                          (gchar*)signal->user_data);
}

void QuidditySignalSubscriber::set_manager_impl(QuiddityManager_Impl::ptr manager_impl) {
  manager_impl_ = manager_impl;
}

void QuidditySignalSubscriber::set_user_data(void* user_data) { user_data_ = user_data; }

void QuidditySignalSubscriber::set_name(const gchar* name) { name_ = name; }

void QuidditySignalSubscriber::set_callback(OnEmittedCallback cb) { user_callback_ = cb; }

bool QuidditySignalSubscriber::subscribe(Quiddity::ptr quid, const std::string& signal_name) {
  if (!quid || user_callback_ == nullptr) {
    g_warning("cannot subscribe to signal (%s %s)", quid->get_name().c_str(), signal_name.c_str());
    return false;
  }
  std::pair<std::string, std::string> cur_pair;
  cur_pair = std::make_pair(quid->get_name(), signal_name);
  if (signal_datas_.find(cur_pair) != signal_datas_.end()) {
    g_warning("not subscribing twice the same signal (%s %s)",
              quid->get_name().c_str(),
              signal_name.c_str());
    return false;
  }
  SignalData* signal = new SignalData();
  signal->subscriber = this;
  signal->name = name_;
  signal->quiddity_name = quid->get_name();
  signal->signal_name = signal_name;
  signal->user_callback = user_callback_;
  signal->user_data = user_data_;
  signal->quid = quid;
  if (quid->subscribe_signal(signal_name.c_str(), signal_cb, signal)) {
    signal_datas_[cur_pair] = signal;
    return true;
  }
  g_warning("QuidditySignalSubscriber: cannot subscribe to signal");
  delete signal;
  return false;
}

bool QuidditySignalSubscriber::unsubscribe(Quiddity::ptr quid, const std::string& signal_name) {
  if (!quid) return false;
  std::pair<std::string, std::string> cur_pair;
  cur_pair = std::make_pair(quid->get_name(), signal_name);
  SignalDataMap::iterator it = signal_datas_.find(cur_pair);
  if (it != signal_datas_.end()) {
    quid->unsubscribe_signal(signal_name, signal_cb, it->second);
    delete (it->second);
    signal_datas_.erase(cur_pair);
    return true;
  }
  g_warning("not unsubscribing a not registered signal (%s %s)",
            quid->get_name().c_str(),
            signal_name.c_str());
  return false;
}

bool QuidditySignalSubscriber::unsubscribe(Quiddity::ptr quid) {
  std::string quid_name = quid->get_name();
  std::vector<std::string> sigs_to_unregister;
  for (auto& it : signal_datas_)
    if (it.first.first == quid_name) sigs_to_unregister.push_back(it.first.second);
  for (auto& it : sigs_to_unregister) unsubscribe(quid, it);
  return true;
}

std::vector<std::pair<std::string, std::string>>
QuidditySignalSubscriber::list_subscribed_signals() {
  std::vector<std::pair<std::string, std::string>> res;
  for (auto& it : signal_datas_) {
    res.push_back(it.first);
  }
  return res;
}

}  // namespace switcher
