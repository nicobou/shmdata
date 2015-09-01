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
#include "./std2.hpp"

namespace switcher {
class PContainer{
 public:
  using prop_id_t = PropertyBase::prop_id_t;
  using notify_cb_t = PropertyBase::notify_cb_t;
  using register_id_t = PropertyBase::register_id_t;
  PContainer() = delete;
  PContainer(data::Tree::ptr tree);  // will own it and write into .property.

  prop_id_t make_int(const std::string &strid,
                     Property2<int>::set_cb_t set,
                     Property2<int>::get_cb_t get,
                     const std::string &label,
                     const std::string &description,
                     int default_value,
                     int min,
                     int max);

  prop_id_t make_parented_int(const std::string &strid,
                              const std::string &parent_strid,
                              Property2<int>::set_cb_t set,
                              Property2<int>::get_cb_t get,
                              const std::string &label,
                              const std::string &description,
                              int default_value,
                              int min,
                              int max);

  prop_id_t make_unsigned_int(const std::string &strid,
                              Property2<unsigned int>::set_cb_t set,
                              Property2<unsigned int>::get_cb_t get,
                              const std::string &label,
                              const std::string &description,
                              unsigned int default_value,
                              unsigned int min,
                              unsigned int max);

  prop_id_t make_parented_unsigned_int(const std::string &strid,
                                       const std::string &parent_strid,
                                       Property2<unsigned int>::set_cb_t set,
                                       Property2<unsigned int>::get_cb_t get,
                                       const std::string &label,
                                       const std::string &description,
                                       unsigned int default_value,
                                       unsigned int min,
                                       unsigned int max);

  prop_id_t make_bool(const std::string &strid,
                     Property2<bool>::set_cb_t set,
                     Property2<bool>::get_cb_t get,
                     const std::string &label,
                     const std::string &description,
                     bool default_value);

  prop_id_t make_parented_bool(const std::string &strid,
                              const std::string &parent_strid,
                              Property2<bool>::set_cb_t set,
                              Property2<bool>::get_cb_t get,
                              const std::string &label,
                              const std::string &description,
                              bool default_value);

  prop_id_t make_float(const std::string &strid,
                     Property2<float>::set_cb_t set,
                     Property2<float>::get_cb_t get,
                     const std::string &label,
                     const std::string &description,
                     float default_value,
                     float min,
                     float max);

  prop_id_t make_parented_float(const std::string &strid,
                              const std::string &parent_strid,
                              Property2<float>::set_cb_t set,
                              Property2<float>::get_cb_t get,
                              const std::string &label,
                              const std::string &description,
                              float default_value,
                              float min,
                              float max);

  prop_id_t make_double(const std::string &strid,
                     Property2<double>::set_cb_t set,
                     Property2<double>::get_cb_t get,
                     const std::string &label,
                     const std::string &description,
                     double default_value,
                     double min,
                     double max);

  prop_id_t make_parented_double(const std::string &strid,
                              const std::string &parent_strid,
                              Property2<double>::set_cb_t set,
                              Property2<double>::get_cb_t get,
                              const std::string &label,
                              const std::string &description,
                              double default_value,
                              double min,
                              double max);

  prop_id_t make_string(const std::string &strid,
                        Property2<std::string>::set_cb_t set,
                        Property2<std::string>::get_cb_t get,
                        const std::string &label,
                        const std::string &description,
                        std::string default_value);

  prop_id_t make_parented_string(const std::string &strid,
                                 const std::string &parent_strid,
                                 Property2<std::string>::set_cb_t set,
                                 Property2<std::string>::get_cb_t get,
                                 const std::string &label,
                                 const std::string &description,
                                 std::string default_value);

  prop_id_t make_selection(const std::string &strid,
                           Property2<Selection, size_t>::set_cb_t set,
                           Property2<Selection, size_t>::get_cb_t get,
                           const std::string &label,
                           const std::string &description,
                           const Selection &default_value);

  prop_id_t make_parented_selection(const std::string &strid,
                                    const std::string &parent_strid,
                                    Property2<Selection, size_t>::set_cb_t set,
                                    Property2<Selection, size_t>::get_cb_t get,
                                    const std::string &label,
                                    const std::string &description,
                                    const Selection &default_value);

  prop_id_t make_label(const std::string &strid,
                       const std::string &label,
                       const std::string &description);

  prop_id_t make_parented_label(const std::string &strid,
                                const std::string &parent_strid,
                                const std::string &label,
                                const std::string &description);

  
  // TODO bool remake(prop_id_t prop_id);

  bool remove(prop_id_t prop_id);
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
    return static_cast<Property2<T> *>(prop_it->second.get())->get();
  }

  
 private:
  prop_id_t counter_{0};
  std::map<prop_id_t, std::unique_ptr<PropertyBase>> props_{};
  std::map<std::string, id_t> ids_{};
  std::map<id_t, std::string> strids_{};
  data::Tree::ptr tree_;
  CounterMap suborders_{};

  template<typename PropType,
           typename PropGetSet = PropType,
           typename ...PropArgs>
  prop_id_t make_under_parent(const std::string &strid,
                              const std::string &parent_strid,
                              PropArgs ...args){
    if(ids_.cend() != ids_.find(strid))
      return 0;  // strid already taken
    if(parent_strid != "" && ids_.cend() == ids_.find(parent_strid))
      return 0;  // parent not found
    props_[++counter_] =
        std2::make_unique<Property2<PropType, PropGetSet>>(std::forward<PropArgs>(args)...);
    ids_[strid] = counter_;
    strids_[counter_] = strid;
    auto *prop = props_[counter_].get();
    prop->set_id(counter_);
    auto tree = prop->get_spec();
    tree_->graft(std::string("property.") + strid, tree);
    tree->graft("id", data::Tree::make(strid));
    tree->graft("order", data::Tree::make(20 * (suborders_.get_count(parent_strid) + 1)));
    tree->graft("parent", data::Tree::make(parent_strid));
    tree->graft("enabled", data::Tree::make(true));
    return counter_;
  }
};

}  // namespace switcher
#endif
