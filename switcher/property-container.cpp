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

bool PContainer::install(PropertyBase *prop,
                         const std::string &strid){
  return install_full(prop,
                      strid,
                      "",
                      20 * (counter_ + 1));
}

bool PContainer::install_under_parent(PropertyBase *parent,
                                      PropertyBase *prop,
                                      const std::string &strid){
  //finding parent strid
  auto parent_id = parent->get_id();
  if (0 == parent_id)
    return false;  // parent has not been installed
  const auto &it = strids_.find(parent_id);
  if (strids_.end() == it)
    return false;  // bug
  // installing
  return install_full(prop,
                      strid,
                      it->second,
                      20 * (suborders_.get_count(it->second) + 1));
}

bool PContainer::install_full(PropertyBase *prop,
                              const std::string &strid,
                              const std::string &parent_strid,
                              size_t order){
  if(ids_.cend() != ids_.find(strid))
    return false;  // strid already taken
  props_[++counter_] = prop;
  ids_[strid] = counter_;
  strids_[counter_] = strid;
  prop->set_id(counter_);
  auto tree = prop->get_spec();
  tree_->graft(std::string("property.") + strid, tree);
  tree->graft("id", data::Tree::make(strid));
  tree->graft("order", data::Tree::make(order));
  tree->graft("parent", data::Tree::make(parent_strid));
  tree->graft("enabled", data::Tree::make(true));
  return true;
}

bool PContainer::reinstall(prop_id_t prop_id,
                           PropertyBase *prop){
  props_[prop_id] = prop;
  prop->set_id(counter_);
  return true;
}

bool PContainer::uninstall(prop_id_t prop_id){
  if (0 == prop_id)
    return false;
  props_[prop_id]->set_id(0);
  props_.erase(prop_id);
  auto it = std::find_if(ids_.begin(),
                         ids_.end(),
                         [&](const std::pair<std::string, prop_id_t> &it){
                           return it.second == prop_id;
                         });
  if (ids_.end() != it)
    ids_.erase(it);
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
