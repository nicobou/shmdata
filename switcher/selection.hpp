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

#ifndef __SWITCHER_SELECTION_H__
#define __SWITCHER_SELECTION_H__

#include <glib.h>
#include <algorithm>
#include <string>
#include <vector>

namespace switcher {
template <typename T = std::string>
class Selection {
 public:
  using index_t = size_t;

  Selection() = delete;

  Selection(std::vector<std::string>&& list, index_t selection)
      : list_(list), attached_(list), current_selection_(selection) {}

  Selection(std::pair<std::vector<std::string> /*names*/, std::vector<T /*attached*/>>&& list,
            index_t selection)
      : list_(std::get<0>(list)), attached_(std::get<1>(list)), current_selection_(selection) {}

  Selection(std::vector<std::string>&& names, std::vector<T>&& attached, index_t selection)
      : list_(names), attached_(attached), current_selection_(selection) {}

  void select(index_t new_selection) { current_selection_ = new_selection; }

  index_t get() const { return current_selection_; }

  std::string get_current() const { return list_[current_selection_]; }

  T get_attached() const { return attached_[current_selection_]; }

  std::vector<std::string> get_list() const { return list_; }

  index_t get_index(const std::string& name_or_attached) {
    {
      const auto& it = std::find(list_.cbegin(), list_.cend(), name_or_attached);
      if (it != list_.end()) return it - list_.begin();
    }
    {
      const auto& it = std::find(attached_.cbegin(), attached_.cend(), name_or_attached);
      if (it != attached_.end()) return it - attached_.begin();
    }
    g_warning("index not found for selection named %s", name_or_attached.c_str());
    return 0;
  }

  index_t size() const { return list_.size(); }

  bool empty() const { return list_.empty(); }

 private:
  std::vector<std::string> list_;
  std::vector<T> attached_;
  index_t current_selection_{0};
};

}  // namespace switcher
#endif
