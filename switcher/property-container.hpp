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

#ifndef __SWITCHER_PROPERTY_CONTAINER_H__
#define __SWITCHER_PROPERTY_CONTAINER_H__

#include <glib.h>  // logs
#include <string>
#include <map>
#include "./property2.hpp"
#include "./counter-map.hpp"

namespace switcher {
class PContainer{
 public:
  using prop_id_t = PropertyBase::prop_id_t;
  using notify_cb_t = PropertyBase::notify_cb_t;
  using register_id_t = PropertyBase::register_id_t;
  PContainer() = delete;
  PContainer(data::Tree::ptr tree);  // will own it and write into .property.
  bool install(PropertyBase *prop,
               const std::string &strid);
  bool install_under_parent(PropertyBase *parent,
                            PropertyBase *prop,
                            const std::string &strid);
  bool reinstall(prop_id_t prop_id, PropertyBase *prop);
  bool uninstall(prop_id_t prop_id);
  bool enable(prop_id_t prop_id, bool enable);

  // return 0 if id is not found
  prop_id_t get_id_from_string_id(const std::string &id) const;

  register_id_t subscribe(prop_id_t id, notify_cb_t fun);
  bool unsubscribe(prop_id_t id, register_id_t rid);

  template<typename T> bool set(prop_id_t id, const T &val){
    auto &prop_it = props_.find(id); 
    if (prop_it->second->get_type_id_hash() != typeid(val).hash_code()){
      g_warning("%s: types do not match", __FUNCTION__);
      return false;
    }
    return static_cast<Property2<T> *>(prop_it->second)->set(std::forward<const T &>(val));
  }

  template<typename T> T get(prop_id_t id) const{
    const auto &prop_it = props_.find(id);
    if (prop_it->second->get_type_id_hash() != typeid(T).hash_code()){
      g_warning("%s: types do not match", __FUNCTION__);
    }
    return static_cast<Property2<T> *>(prop_it->second)->get();
  }

 private:
  prop_id_t counter_{0};
  std::map<prop_id_t, PropertyBase *> props_{};
  std::map<std::string, id_t> ids_{};
  std::map<id_t, std::string> strids_{};
  data::Tree::ptr tree_;
  CounterMap suborders_{};

  bool install_full(PropertyBase *prop,
                    const std::string &strid,
                    const std::string &parent_strid,
                    size_t order);

};

}  // namespace switcher
#endif
