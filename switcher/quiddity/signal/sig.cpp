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

#include "./sig.hpp"

namespace switcher {

Sig::register_id_t Sig::subscribe(notify_cb_t fun) const {
  to_notify_[++counter_] = fun;
  return counter_;
}

bool Sig::unsubscribe(register_id_t rid) const {
  auto it = to_notify_.find(rid);
  if (to_notify_.end() == it) return false;
  to_notify_.erase(it);
  return true;
}

void Sig::notify(InfoTree::ptr tree) const {
  for (auto& it : to_notify_) it.second(tree);
}

}  // namespace switcher
