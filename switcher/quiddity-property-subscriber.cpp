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
 * The Quiddity property subscriber
 */

#include "./quiddity-property-subscriber.hpp"
#include "./quiddity.hpp"
#include "./quiddity-manager-impl.hpp"

namespace switcher {
QuiddityPropertySubscriber::QuiddityPropertySubscriber() {
}

QuiddityPropertySubscriber::~QuiddityPropertySubscriber() {
}

void QuiddityPropertySubscriber::mute(bool muted) {
  muted_ = muted;
}

void
QuiddityPropertySubscriber::property_cb(GObject *gobject,
                                        GParamSpec *pspec,
                                        gpointer user_data) {
  PropertyData *prop = static_cast<PropertyData *>(user_data);
  if (!prop->property_subscriber->muted_)
    prop->user_callback(prop->name,
                        prop->quiddity_name,
                        prop->property_name,
                        Property::parse_callback_args(gobject, pspec),
                        (gchar *) prop->user_data);
}

void
QuiddityPropertySubscriber::
set_manager_impl(QuiddityManager_Impl::ptr manager_impl) {
  manager_impl_ = manager_impl;
}

void QuiddityPropertySubscriber::set_user_data(void *user_data) {
  user_data_ = user_data;
}

void QuiddityPropertySubscriber::set_name(const gchar *name) {
  name_ = name;
}

void QuiddityPropertySubscriber::set_callback(Callback cb) {
  user_callback_ = cb;
}

bool
QuiddityPropertySubscriber::subscribe(Quiddity::ptr quid,
                                      const std::string &property_name) {
  if (!quid || user_callback_ == nullptr) {
    g_warning("cannot subscribe (%s %s)",
              quid->get_name().c_str(), property_name.c_str());
    return false;
  }
  auto cur_pair = std::make_pair(quid->get_name(), property_name);
  if (prop_datas_.find(cur_pair) != prop_datas_.end()) {
    g_warning("not subscribing twice the same property (%s %s)",
              quid->get_name().c_str(), property_name.c_str());
    return false;
  }
  PropertyData *prop = new PropertyData(); 
  prop->property_subscriber = this;
  prop->name = g_strdup(name_.c_str());
  prop->quiddity_name = g_strdup(quid->get_name().c_str());
  prop->property_name = g_strdup(property_name.c_str());
  prop->user_callback = user_callback_;
  prop->user_data = user_data_;
  prop->quid = quid;
  if (quid->subscribe_property(property_name.c_str(), property_cb, prop)) {
    prop_datas_[cur_pair] = prop;
    return true;
  }
  g_free(prop->name);
  g_free(prop->quiddity_name);
  g_free(prop->property_name);
  delete prop;
  g_warning("QuiddityPropertySubscriber: cannot subscribe to property");
  return false;
}

bool
QuiddityPropertySubscriber::unsubscribe(Quiddity::ptr quid,
                                        const std::string &property_name) {
  if (!quid)
    return false;
  auto cur_pair = std::make_pair(quid->get_name(), property_name);
  PropDataMap::iterator it = prop_datas_.find(cur_pair);
  if (it != prop_datas_.end()) {
    if(!quid->unsubscribe_property(property_name, property_cb, it->second)) {
      g_warning("cannot unsubscribe to %s", property_name.c_str());
      return false;
    }
    g_free(it->second->name);
    g_free(it->second->quiddity_name);
    g_free(it->second->property_name);
    delete it->second;
    prop_datas_.erase(cur_pair);
    return true;
  }
  g_warning("not unsubscribing a not registered property (%s %s)",
            quid->get_name().c_str(), property_name.c_str());
  return false;
}

bool QuiddityPropertySubscriber::unsubscribe(Quiddity::ptr quid) {
  auto quid_name = quid->get_name();
  std::vector<std::pair<std::string, std::string>> entries_to_remove;
  for (auto &it : prop_datas_)
    if (it.first.first == quid_name) {
      if(!quid->unsubscribe_property(it.second->property_name,
                                     property_cb,
                                     it.second)) {
        g_warning("cannot unsubscribe property for quiddity %s",
                  quid_name.c_str());
        return false;
      }
      g_free(it.second->name);
      g_free(it.second->quiddity_name);
      g_free(it.second->property_name);
      delete it.second;
      entries_to_remove.push_back(it.first);
    }
  for (auto &it : entries_to_remove)
    prop_datas_.erase(it);
  return true;
}

std::vector<std::pair<std::string, std::string>>
    QuiddityPropertySubscriber::list_subscribed_properties() {
  std::vector<std::pair<std::string, std::string>> res;
  for (auto &it : prop_datas_)
    res.push_back(it.first);
  return res;
}
}
