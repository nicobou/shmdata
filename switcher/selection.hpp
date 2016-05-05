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
  using index_t = size_t;
  Selection() = delete;
  Selection(std::vector<std::string>&& list, index_t selection);
  Selection(std::pair<std::vector<std::string> /*names*/,
                      std::vector<std::string /*nicks*/>>&& list,
            index_t selection);
  void select(index_t new_selection);
  index_t get() const;
  std::string get_current() const;
  std::string get_current_nick() const;
  std::vector<std::string> get_list() const;
  index_t get_index(const std::string& name_or_nick);
  index_t size() const { return list_.size(); }
  bool empty() const { return list_.empty(); }

 private:
  std::vector<std::string> list_;
  std::vector<std::string> nicks_;
  index_t current_selection_{0};
};

}  // namespace switcher
#endif
