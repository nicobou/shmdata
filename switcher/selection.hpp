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
#include <cctype>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace switcher {
// The following struct allows the select method to be invoked by index or by name
struct IndexOrName {
  IndexOrName() = delete;
  IndexOrName(size_t idx) : index_(idx), is_index_(true) {}
  IndexOrName(std::string str) : name_(str), is_index_(false) {}
  IndexOrName(size_t idx, std::string str) : index_(idx), name_(str), is_index_(true) {}
  std::string to_string() const { return std::to_string(index_); }
  static std::pair<bool, IndexOrName> from_string(const std::string& str) {
    if (!isdigit(*str.begin()) && !('-' == *str.begin() && !isdigit(*str.begin())))
      return std::make_pair(true, switcher::IndexOrName(str));
    std::istringstream iss(str);
    size_t res;
    iss >> res;
    return std::make_pair(true, switcher::IndexOrName(res));
  }

  const size_t index_{0};
  const std::string name_{};
  const bool is_index_;
};

template <typename T = std::string>
class Selection {
 public:
  using index_t = size_t;

  Selection() = delete;

  Selection(std::vector<std::string>&& list, index_t selection)
      : list_(list), attached_(list_), current_selection_(selection) {}

  Selection(std::pair<std::vector<std::string> /*names*/, std::vector<T /*attached*/>>&& list,
            index_t selection)
      : list_(std::get<0>(list)), attached_(std::get<1>(list)), current_selection_(selection) {}

  Selection(std::vector<std::string>&& names, std::vector<T>&& attached, index_t selection)
      : list_(names), attached_(attached), current_selection_(selection) {}

  void select(IndexOrName new_selection) {
    if (new_selection.is_index_) {
      current_selection_ = new_selection.index_;
    } else {
      current_selection_ = get_name_index(new_selection.name_);
    }
  }

  IndexOrName get() const { return IndexOrName(current_selection_, get_current()); }

  size_t get_current_index() const { return current_selection_; }

  std::string get_current() const { return list_[current_selection_]; }

  T get_attached() const { return attached_[current_selection_]; }

  std::vector<std::string> get_list() const { return list_; }

  index_t get_name_index(const std::string& name) {
    {
      const auto& it = std::find(list_.cbegin(), list_.cend(), name);
      if (it != list_.end()) return it - list_.begin();
    }
    return current_selection_;
  }

  index_t get_index(const std::string& name_or_attached) {
    {
      const auto& it = std::find(list_.cbegin(), list_.cend(), name_or_attached);
      if (it != list_.end()) return it - list_.begin();
    }
    {
      const auto& it = std::find(attached_.cbegin(), attached_.cend(), name_or_attached);
      if (it != attached_.end()) return it - attached_.begin();
    }
    return current_selection_;
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
