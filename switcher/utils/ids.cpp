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

#include "./ids.hpp"

#include <algorithm>
#include <limits>

namespace switcher {

const Ids::id_t Ids::kInvalid = 0;
const Ids::id_t Ids::kMaxNumOfIds =
    std::numeric_limits<id_t>::max() - std::numeric_limits<id_t>::min() - 1;

Ids::id_t Ids::allocate_id() {
  if (ids_.size() == kMaxNumOfIds) return kInvalid;
  do {
    if (cur_id_ == std::numeric_limits<id_t>::max()) {
      cur_id_ = 1;
    } else {
      ++cur_id_;
    }
  } while (ids_.cend() != std::find(std::cbegin(ids_), std::cend(ids_), cur_id_));
  ids_.emplace_back(cur_id_);
  return cur_id_;
}

void Ids::release_last_allocated_id() { ids_.erase(ids_.end() - 1); }

bool Ids::release_id(id_t id) {
  auto it = std::find(ids_.begin(), ids_.end(), id);
  if (ids_.end() == it) {
    return false;
  }
  ids_.erase(it);
  return true;
}

std::vector<Ids::id_t> Ids::get_ids() const { return ids_; }

bool Ids::is_allocated(id_t id) const {
  return ids_.end() != std::find(ids_.begin(), ids_.end(), id);
}

}  // namespace switcher
