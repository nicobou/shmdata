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
#include "./property-internal-types.hpp"
#include "./counter-map.hpp"
#include "./std2.hpp"

namespace switcher {
class PContainer{
 public:
  using prop_id_t = PropertyBase::prop_id_t;
  using notify_cb_t = PropertyBase::notify_cb_t;
  using register_id_t = PropertyBase::register_id_t;
  enum pstate_t {ENABLED, DISABLED, REMOVED};
  using pstate_cb_t = std::function<void(pstate_t state)>;
  PContainer() = delete;
  PContainer(data::Tree::ptr tree);  // will own it and write into .property.

  prop_id_t make_int(const std::string &strid,
                     prop::set_t<int> set,
                     prop::get_t<int> get,
                     const std::string &label,
                     const std::string &description,
                     int default_value,
                     int min,
                     int max);

  prop_id_t make_parented_int(const std::string &strid,
                              const std::string &parent_strid,
                              prop::set_t<int> set,
                              prop::get_t<int> get,
                              const std::string &label,
                              const std::string &description,
                              int default_value,
                              int min,
                              int max);

  prop_id_t make_short(const std::string &strid,
                       prop::set_t<short> set,
                       prop::get_t<short> get,
                       const std::string &label,
                       const std::string &description,
                       short default_value,
                       short min,
                       short max);

  prop_id_t make_parented_short(const std::string &strid,
                                const std::string &parent_strid,
                                prop::set_t<short> set,
                                prop::get_t<short> get,
                                const std::string &label,
                                const std::string &description,
                                short default_value,
                                short min,
                                short max);

  prop_id_t make_long(const std::string &strid,
                      prop::set_t<long> set,
                      prop::get_t<long> get,
                      const std::string &label,
                      const std::string &description,
                      long default_value,
                      long min,
                      long max);

  prop_id_t make_parented_long(const std::string &strid,
                               const std::string &parent_strid,
                               prop::set_t<long> set,
                               prop::get_t<long> get,
                               const std::string &label,
                               const std::string &description,
                               long default_value,
                               long min,
                               long max);

  prop_id_t make_long_long(const std::string &strid,
                           Property2<long long>::set_cb_t set,
                           Property2<long long>::get_cb_t get,
                           const std::string &label,
                           const std::string &description,
                           long long default_value,
                           long long min,
                           long long max);
  
  prop_id_t make_parented_long_long(const std::string &strid,
                                    const std::string &parent_strid,
                                    Property2<long long>::set_cb_t set,
                                    Property2<long long>::get_cb_t get,
                                    const std::string &label,
                                    const std::string &description,
                                    long long default_value,
                                    long long min,
                                    long long max);
    
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

  prop_id_t make_unsigned_short(const std::string &strid,
                                Property2<unsigned short>::set_cb_t set,
                                Property2<unsigned short>::get_cb_t get,
                                const std::string &label,
                                const std::string &description,
                                unsigned short default_value,
                                unsigned short min,
                                unsigned short max);

  prop_id_t make_parented_unsigned_short(const std::string &strid,
                                         const std::string &parent_strid,
                                         Property2<unsigned short>::set_cb_t set,
                                         Property2<unsigned short>::get_cb_t get,
                                         const std::string &label,
                                         const std::string &description,
                                         unsigned short default_value,
                                         unsigned short min,
                                         unsigned short max);

  prop_id_t make_unsigned_long(const std::string &strid,
                               Property2<unsigned long>::set_cb_t set,
                               Property2<unsigned long>::get_cb_t get,
                               const std::string &label,
                               const std::string &description,
                               unsigned long default_value,
                               unsigned long min,
                               unsigned long max);

  prop_id_t make_parented_unsigned_long(const std::string &strid,
                                        const std::string &parent_strid,
                                        Property2<unsigned long>::set_cb_t set,
                                        Property2<unsigned long>::get_cb_t get,
                                        const std::string &label,
                                        const std::string &description,
                                        unsigned long default_value,
                                        unsigned long min,
                                        unsigned long max);

  prop_id_t make_unsigned_long_long(const std::string &strid,
                                    Property2<unsigned long long>::set_cb_t set,
                                    Property2<unsigned long long>::get_cb_t get,
                                    const std::string &label,
                                    const std::string &description,
                                    unsigned long long default_value,
                                    unsigned long long min,
                                    unsigned long long max);

  prop_id_t make_parented_unsigned_long_long(const std::string &strid,
                                             const std::string &parent_strid,
                                             Property2<unsigned long long>::set_cb_t set,
                                             Property2<unsigned long long>::get_cb_t get,
                                             const std::string &label,
                                             const std::string &description,
                                             unsigned long long default_value,
                                             unsigned long long min,
                                             unsigned long long max);
  
  prop_id_t make_bool(const std::string &strid,
                      prop::set_t<bool> set,
                      prop::get_t<bool> get,
                      const std::string &label,
                      const std::string &description,
                      bool default_value);
  
  prop_id_t make_parented_bool(const std::string &strid,
                               const std::string &parent_strid,
                               prop::set_t<bool> set,
                               prop::get_t<bool> get,
                               const std::string &label,
                               const std::string &description,
                               bool default_value);
  
  prop_id_t make_float(const std::string &strid,
                       prop::set_t<float> set,
                       prop::get_t<float> get,
                       const std::string &label,
                       const std::string &description,
                       float default_value,
                       float min,
                       float max);
  
  prop_id_t make_parented_float(const std::string &strid,
                                const std::string &parent_strid,
                                prop::set_t<float> set,
                                prop::get_t<float> get,
                                const std::string &label,
                                const std::string &description,
                                float default_value,
                                float min,
                                float max);
  
  prop_id_t make_double(const std::string &strid,
                        prop::set_t<double> set,
                        prop::get_t<double> get,
                        const std::string &label,
                        const std::string &description,
                        double default_value,
                        double min,
                        double max);

  prop_id_t make_parented_double(const std::string &strid,
                                 const std::string &parent_strid,
                                 prop::set_t<double> set,
                                 prop::get_t<double> get,
                                 const std::string &label,
                                 const std::string &description,
                                 double default_value,
                                 double min,
                                 double max);

  prop_id_t make_long_double(const std::string &strid,
                             Property2<long double>::set_cb_t set,
                             Property2<long double>::get_cb_t get,
                             const std::string &label,
                             const std::string &description,
                             long double default_value,
                             long double min,
                             long double max);

  prop_id_t make_parented_long_double(const std::string &strid,
                                      const std::string &parent_strid,
                                      Property2<long double>::set_cb_t set,
                                      Property2<long double>::get_cb_t get,
                                      const std::string &label,
                                      const std::string &description,
                                      long double default_value,
                                      long double min,
                                      long double max);

  prop_id_t make_char(const std::string &strid,
                      prop::set_t<char> set,
                      prop::get_t<char> get,
                      const std::string &label,
                      const std::string &description,
                      char default_value);

  prop_id_t make_parented_char(const std::string &strid,
                               const std::string &parent_strid,
                               prop::set_t<char> set,
                               prop::get_t<char> get,
                               const std::string &label,
                               const std::string &description,
                               char default_value);


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

  prop_id_t make_group(const std::string &strid,
                       const std::string &label,
                       const std::string &description);

  prop_id_t make_parented_group(const std::string &strid,
                                const std::string &parent_strid,
                                const std::string &label,
                                const std::string &description);

  prop_id_t make_fraction(const std::string &strid,
                          prop::set_t<Fraction> set,
                          prop::get_t<Fraction> get,
                          const std::string &label,
                          const std::string &description,
                          const Fraction &default_value,
                          Fraction::ator_t min_num,
                          Fraction::ator_t min_denom,
                          Fraction::ator_t max_num,
                          Fraction::ator_t max_denom);
  
  prop_id_t make_parented_fraction(const std::string &strid,
                                    const std::string &parent_strid,
                                    prop::set_t<Fraction> set,
                                    prop::get_t<Fraction> get,
                                    const std::string &label,
                                    const std::string &description,
                                    const Fraction &default_value,
                          Fraction::ator_t min_num,
                          Fraction::ator_t min_denom,
                          Fraction::ator_t max_num,
                          Fraction::ator_t max_denom);


  template<typename ...T>
  prop_id_t make_tuple(const std::string &strid,
                       std::function<bool(const std::tuple<T...> &)> set,
                       std::function<std::tuple<T...>()> get,
                       const std::string &label,
                       const std::string &description,
                       const std::tuple<T...> &default_value){
    return make_under_parent<std::tuple<T...>>(
        strid, "", set, get, label, description,
        std::forward<const std::tuple<T...> &>(default_value));
  }

  template<typename ...T>
  prop_id_t make_parented_tuple(const std::string &strid,
                                const std::string &parent_strid,
                                std::function<bool(const std::tuple<T...> &)> set,
                                std::function<std::tuple<T...>()> get,
                                const std::string &label,
                                const std::string &description,
                                const std::tuple<T...> &default_value){
    return make_under_parent<std::tuple<T...>>(
        strid, parent_strid, set, get, label, description,
        std::forward<const std::tuple<T...> &>(default_value));
  }

  
  // TODO bool remake(prop_id_t prop_id);

  bool remove(prop_id_t prop_id);
  bool enable(prop_id_t prop_id, bool enable);

  // return 0 if id is not found
  prop_id_t get_id_from_string_id(const std::string &id) const;

  register_id_t subscribe(prop_id_t id, notify_cb_t fun, pstate_cb_t state_cb) const;
  bool unsubscribe(prop_id_t id, register_id_t rid) const;

  bool set_str(prop_id_t id, const std::string &val) const{
    if(0 == id)
      return false;
    auto prop_it = props_.find(id); 
    return prop_it->second.get()->set_str(std::forward<const std::string &>(val));
  }

  std::string get_str(prop_id_t id) const{
    auto prop_it = props_.find(id); 
    return prop_it->second.get()->get_str();
  }

  bool set_str_str(const std::string &strid, const std::string &val) const{
    auto id = get_id_from_string_id(strid);
    if(0 == id)
      return false;
    auto prop_it = props_.find(id); 
    return prop_it->second.get()->set_str(std::forward<const std::string &>(val));
  }

  std::string get_str_str(const std::string &strid) const{
    auto id = get_id_from_string_id(strid);
    if(0 == id)
      return std::string();
    auto prop_it = props_.find(id); 
    return prop_it->second.get()->get_str();
  }

  template<typename T> bool set(prop_id_t id, const T &val) const{
    auto prop_it = props_.find(id); 
    if (prop_it->second->get_type_id_hash() != typeid(val).hash_code()){
      g_warning("%s: types do not match", __FUNCTION__);
      return false;
    }
    return static_cast<Property2<T> *>(prop_it->second.get())->
        set(std::forward<const T &>(val));
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
  mutable std::unordered_map<register_id_t, pstate_cb_t> state_cbs_{};
  
  void notify_state_change(prop_id_t prop, pstate_t state);
  
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
