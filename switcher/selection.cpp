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

#include <glib.h>
#include "./selection.hpp"

namespace switcher {

Selection::Selection(std::vector<std::string> &&list, index_t current_selection):
    list_(list),
    current_selection_(current_selection){
}

void Selection::select(index_t current_selection){
  if (current_selection >= list_.size()){
    g_warning("current_selection >= list_.size()");
    return;
  }
  current_selection_ = current_selection;
}

Selection::index_t Selection::get() const{
  return current_selection_;
}

std::vector<std::string> Selection::get_list() const{
  return list_;
}
 
}  // namespace switcher
