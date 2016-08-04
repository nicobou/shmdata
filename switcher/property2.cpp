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

#include "./property2.hpp"

namespace switcher {

PropertyBase::PropertyBase(size_t type_hash) : type_hash_(type_hash) {}

PropertyBase::prop_id_t PropertyBase::get_id() const { return id_; }

PropertyBase::register_id_t PropertyBase::subscribe(notify_cb_t fun) const {
  to_notify_[++counter_] = fun;
  return counter_;
}

bool PropertyBase::unsubscribe(register_id_t rid) const {
  auto it = to_notify_.find(rid);
  if (to_notify_.end() == it) return false;
  to_notify_.erase(it);
  return true;
}

size_t PropertyBase::get_type_id_hash() const { return type_hash_; }

void PropertyBase::notify() const {
  for (auto& it : to_notify_) it.second();
}

std::map<PropertyBase::register_id_t, PropertyBase::notify_cb_t> PropertyBase::get_notify_cbs()
    const {
  return to_notify_;
};

void PropertyBase::set_notify_cbs(std::map<register_id_t, notify_cb_t> cbs) { to_notify_ = cbs; };

void PropertyBase::set_id(prop_id_t id) { id_ = id; }

std::vector<PropertyBase::register_id_t> PropertyBase::get_register_ids() const {
  std::vector<register_id_t> res;
  res.reserve(to_notify_.size());
  for (auto& it : to_notify_) res.push_back(it.first);
  return res;
}

}  // namespace switcher
