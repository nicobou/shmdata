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

#include "./property-container.hpp"

namespace switcher {

PContainer::PContainer(data::Tree::ptr tree):
    tree_(tree){
  tree_->graft(".property", data::Tree::make());
  tree_->tag_as_array(".property", true);
}

bool PContainer::remove(prop_id_t prop_id){
  auto it = strids_.find(prop_id);
  if(strids_.end() == it)
    return false;  // prop not found
  tree_->prune(std::string("property.") + it->second);
  ids_.erase(it->second);
  strids_.erase(it);
  props_.erase(prop_id);
  return true;
}

bool PContainer::enable(prop_id_t prop_id, bool enable){
  const auto &it = strids_.find(prop_id);
  if (strids_.end() == it)
    return false;
  tree_->graft(std::string("property.") + it->second + ".enabled", data::Tree::make(enable));
  return true;
}

PContainer::register_id_t PContainer::subscribe(prop_id_t id,
                                                notify_cb_t fun){
  return props_[id]->subscribe(std::forward<notify_cb_t>(fun));
}

bool PContainer::unsubscribe(prop_id_t id,
                             register_id_t rid){
  return props_[id]->unsubscribe(std::forward<register_id_t>(rid));
}

PContainer::prop_id_t PContainer::get_id_from_string_id(const std::string &id) const{
  const auto &it = ids_.find(id);
  if (ids_.end() == it)
    return 0;
  return it->second;
}

}  // namespace switcher
