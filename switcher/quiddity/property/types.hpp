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

#ifndef __SWITCHER_PROPERTY_INTERNAL_TYPES_H__
#define __SWITCHER_PROPERTY_INTERNAL_TYPES_H__

#include <functional>

namespace switcher {
namespace quiddity {
namespace property {

// The following struct allows the select method to be invoked by index or by name
struct IndexOrName {
  IndexOrName() = delete;
  IndexOrName(size_t idx) : index_(idx), is_index_(true) {}
  IndexOrName(std::string str) : name_(str), is_index_(false) {}
  IndexOrName(size_t idx, std::string str) : index_(idx), name_(str), is_index_(true) {}
  std::string to_string() const { return std::to_string(index_); }
  static std::pair<bool, IndexOrName> from_string(const std::string& str) {
    if (!isdigit(*str.begin()) && !('-' == *str.begin() && !isdigit(*str.begin())))
      return std::make_pair(true, IndexOrName(str));
    std::istringstream iss(str);
    size_t res;
    iss >> res;
    return std::make_pair(true, IndexOrName(res));
  }

  const size_t index_{0};
  const std::string name_{};
  const bool is_index_;
};

// set/get
template <typename W>
using get_t = std::function<W()>;

template <typename W>
using set_t = std::function<bool(const W&)>;

// id
using prop_id_t = size_t;
inline prop_id_t id_from_string(const std::string& str) {
  if (!isdigit(*str.begin())) return 0;
  return stoul(str, nullptr, 0);
}

// register
using register_id_t = size_t;
using notify_cb_t = std::function<void()>;

}  // namespace property
}  // namespace quiddity
}  // namespace switcher
#endif
