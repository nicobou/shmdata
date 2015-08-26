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

PropertyContainer::PropertyContainer(data::Tree::ptr tree):
    tree_(tree){
  tree_->graft(".property", data::Tree::make());
  tree_->tag_as_array(".property", true);
}

PropertyContainer::prop_id_t PropertyContainer::install_property(const std::string &name,
                                                                 PropertyBase *prop){
  props_[++counter_] = prop;
  ids_[name] = counter_;
  auto tree = prop->get_spec();
  tree->graft("id", data::Tree::make(name));
  tree_->graft(std::string("property.") + name, tree);
  return counter_;
}

bool PropertyContainer::reinstall_property(prop_id_t prop_id,
                                           PropertyBase *prop){
  props_[prop_id] = prop; // FIXME check in disabled, same for other methods
  return true;
}

bool PropertyContainer::uninstall_property(prop_id_t prop_id){
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

bool PropertyContainer::disable_property(prop_id_t prop_id){
  disabled_props_[prop_id] = props_[prop_id];
  props_.erase(prop_id);
  return true;
}

bool PropertyContainer::enable_property(prop_id_t prop_id){
  props_[prop_id] = disabled_props_[prop_id];
  disabled_props_.erase(prop_id);
  return true;
}

PropertyBase::register_id_t PropertyContainer::subscribe_by_name(const std::string &name,
                                                                 PropertyBase::notify_cb_t fun){
  return props_[ids_[name]]->subscribe(std::forward<PropertyBase::notify_cb_t>(fun));
}

PropertyContainer::prop_id_t PropertyContainer::get_id_from_name(const std::string &name){
  return ids_[name];
}

}  // namespace switcher
