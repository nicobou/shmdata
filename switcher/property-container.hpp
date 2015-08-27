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

#include <string>
#include <map>
#include "./property2.hpp"

namespace switcher {
class PropertyContainer{
 public:
  PropertyContainer() = delete;
  PropertyContainer(data::Tree::ptr tree);  // will own it and write into .property.
  PropertyBase::prop_id_t install_property(const std::string &name, PropertyBase *prop);
  bool reinstall_property(PropertyBase::prop_id_t prop_id, PropertyBase *prop);
  bool uninstall_property(PropertyBase::prop_id_t prop_id);
  bool disable_property(PropertyBase::prop_id_t prop_id);
  bool enable_property(PropertyBase::prop_id_t prop_id);

  // return 0 if name is not found
  PropertyBase::prop_id_t get_id_from_name(const std::string &name) const;

  // FIXME remove other "by name" methods
  PropertyBase::register_id_t subscribe_by_name(const std::string &name,
                                                PropertyBase::notify_cb_t fun);
  template<class T> bool property_set_by_name(const std::string &name, const T &val){
    if (props_[ids_[name]]->get_type_id_hash() != typeid(val).hash_code()){
      std::cerr << "types do not match" << std::endl;  // FIXME
      return false;
    }
    return static_cast<Property2<T> *>(props_[ids_[name]])->
        set(std::forward<const T &>(val));
  }
  // template<class T> T property_get_by_name(const std::string &name) const{
  //   const auto &it = ids_.find(name);
  //   if (ids_.end() == it){
  //     g_warning("name %s not found when getting property", name.c_str());
  //     return T();
  //   }
  //   if (props_[ids_[name]]->get_type_id_hash() != typeid(T).hash_code()){
  //     std::cerr << "types do not match" << std::endl;  //FIXME
  //   }
  //   return static_cast<Property2<T> *>(props_[ids_[name]])->get();
  // }

 private:
  PropertyBase::prop_id_t counter_{0};
  std::map<PropertyBase::prop_id_t, PropertyBase *> props_{};
  std::map<PropertyBase::prop_id_t, PropertyBase *> disabled_props_{};
  std::map<std::string, id_t> ids_{};
  data::Tree::ptr tree_;
};

}  // namespace switcher
#endif
