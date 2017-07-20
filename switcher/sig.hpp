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

#ifndef __SWITCHER_SIG_H__
#define __SWITCHER_SIG_H__

#include <map>
#include "./information-tree.hpp"

namespace switcher {

class Sig {
 public:
  using register_id_t = size_t;
  using notify_cb_t = std::function<void(const InfoTree::ptr&)>;
  using sig_id_t = size_t;
  Sig() = default;

  register_id_t subscribe(notify_cb_t fun) const;
  bool unsubscribe(register_id_t rid) const;
  void notify(InfoTree::ptr tree) const;

 private:
  mutable register_id_t counter_{0};
  mutable std::map<register_id_t, notify_cb_t> to_notify_{};
};

}  // namespace switcher
#endif
