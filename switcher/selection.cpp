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

#include "./selection.hpp"
#include <glib.h>
#include <algorithm>
#include <tuple>

namespace switcher {

Selection::Selection(std::vector<std::string>&& list, index_t current_selection)
    : list_(list), nicks_(list), current_selection_(current_selection) {}

Selection::Selection(std::pair<std::vector<std::string> /*names*/,
                               std::vector<std::string /*nicks*/>>&& list,
                     index_t current_selection)
    : list_(std::get<0>(list)),
      nicks_(std::get<1>(list)),
      current_selection_(current_selection) {}

void Selection::select(index_t current_selection) {
  if (current_selection >= list_.size()) {
    g_warning("current_selection >= list_.size()");
    return;
  }
  current_selection_ = current_selection;
}

Selection::index_t Selection::get() const { return current_selection_; }

std::string Selection::get_current() const { return list_[current_selection_]; }

std::string Selection::get_current_nick() const {
  return nicks_[current_selection_];
}

std::vector<std::string> Selection::get_list() const { return list_; }

Selection::index_t Selection::get_index(const std::string& name_or_nick) {
  {
    const auto& it = std::find(list_.cbegin(), list_.cend(), name_or_nick);
    if (it != list_.end()) return it - list_.begin();
  }
  {
    const auto& it = std::find(nicks_.cbegin(), nicks_.cend(), name_or_nick);
    if (it != nicks_.end()) return it - nicks_.begin();
  }
  g_warning("index not found for selection named %s", name_or_nick.c_str());
  return 0;
}

}  // namespace switcher
