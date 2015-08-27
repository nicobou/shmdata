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

#include <string>
#include <vector>

namespace switcher {
class Selection {
 public:
  Selection() = delete;
  Selection(std::vector<std::string> &&list, size_t current_selection = 0);
  void select(size_t new_selection);
  size_t get() const;
  std::vector<std::string> get_list() const;
  
 private:
  const std::vector<std::string> list_;
  size_t current_selection_{0};
};

}  // namespace switcher
#endif
