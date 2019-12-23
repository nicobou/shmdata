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
namespace prop {

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

}  // namespace propsetget
}  // namespace switcher
#endif
