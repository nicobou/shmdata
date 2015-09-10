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

void PContainer::notify_state_change(prop_id_t prop, pstate_t state) {
  for (const auto &it: props_[prop]->get_register_ids()){
    auto cb = state_cbs_.find(it);
    if (state_cbs_.end() != cb) cb->second(state);
  }
}

bool PContainer::remove(prop_id_t prop_id){
  auto it = strids_.find(prop_id);
  if(strids_.end() == it)
    return false;  // prop not found
  tree_->prune(std::string("property.") + it->second);
  notify_state_change(prop_id, PContainer::REMOVED);
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
  if (enable)
    notify_state_change(prop_id, PContainer::ENABLED);
  else
    notify_state_change(prop_id, PContainer::DISABLED);
  return true;
}

PContainer::register_id_t PContainer::subscribe(
    prop_id_t id,
    notify_cb_t fun,
    pstate_cb_t state_cb) const {
  auto res = props_.find(id)->second->subscribe(std::forward<notify_cb_t>(fun));
  if(nullptr != state_cb)
    state_cbs_[res] = state_cb;
  return res;
}

bool PContainer::unsubscribe(prop_id_t id, register_id_t rid) const{
  return props_.find(id)->second->unsubscribe(std::forward<register_id_t>(rid));
}

PContainer::prop_id_t PContainer::get_id_from_string_id(const std::string &id) const{
  const auto &it = ids_.find(id);
  if (ids_.end() == it)
    return 0;
  return it->second;
}

PContainer::prop_id_t PContainer::make_int(const std::string &strid,
                                           Property2<int>::set_cb_t set,
                                           Property2<int>::get_cb_t get,
                                           const std::string &label,
                                           const std::string &description,
                                           int default_value,
                                           int min,
                                           int max){
  return make_under_parent<int>(strid, "", set, get, label, description,
                                default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_int(const std::string &strid,
                                                    const std::string &parent_strid,
                                                    Property2<int>::set_cb_t set,
                                                    Property2<int>::get_cb_t get,
                                                    const std::string &label,
                                                    const std::string &description,
                                                    int default_value,
                                                    int min,
                                                    int max){
  return make_under_parent<int>(strid, parent_strid, set, get, label, description,
                                default_value, min, max);
}

PContainer::prop_id_t PContainer::make_short(const std::string &strid,
                                             Property2<short>::set_cb_t set,
                                             Property2<short>::get_cb_t get,
                                             const std::string &label,
                                             const std::string &description,
                                             short default_value,
                                             short min,
                                             short max){
  return make_under_parent<short>(strid, "", set, get, label, description,
                                  default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_short(const std::string &strid,
                                                      const std::string &parent_strid,
                                                      Property2<short>::set_cb_t set,
                                                      Property2<short>::get_cb_t get,
                                                      const std::string &label,
                                                      const std::string &description,
                                                      short default_value,
                                                      short min,
                                                      short max){
  return make_under_parent<short>(strid, parent_strid, set, get, label, description,
                                  default_value, min, max);
}

PContainer::prop_id_t PContainer::make_long(const std::string &strid,
                                            Property2<long>::set_cb_t set,
                                            Property2<long>::get_cb_t get,
                                            const std::string &label,
                                            const std::string &description,
                                            long default_value,
                                            long min,
                                            long max){
  return make_under_parent<long>(strid, "", set, get, label, description,
                                 default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_long(const std::string &strid,
                                                     const std::string &parent_strid,
                                                     Property2<long>::set_cb_t set,
                                                     Property2<long>::get_cb_t get,
                                                     const std::string &label,
                                                     const std::string &description,
                                                     long default_value,
                                                     long min,
                                                     long max){
  return make_under_parent<long>(strid, parent_strid, set, get, label, description,
                                 default_value, min, max);
}

PContainer::prop_id_t PContainer::make_long_long(const std::string &strid,
                                                 Property2<long long>::set_cb_t set,
                                                 Property2<long long>::get_cb_t get,
                                                 const std::string &label,
                                                 const std::string &description,
                                                 long long default_value,
                                                 long long min,
                                                 long long max){
  return make_under_parent<long long>(strid, "", set, get, label, description,
                                      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_long_long(const std::string &strid,
                                                          const std::string &parent_strid,
                                                          Property2<long long>::set_cb_t set,
                                                          Property2<long long>::get_cb_t get,
                                                          const std::string &label,
                                                          const std::string &description,
                                                          long long default_value,
                                                          long long min,
                                                          long long max){
  return make_under_parent<long long>(strid, parent_strid, set, get, label, description,
                                      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_int(
    const std::string &strid,
    Property2<unsigned int>::set_cb_t set,
    Property2<unsigned int>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned int default_value,
    unsigned int min,
    unsigned int max){
  return make_under_parent<unsigned int>(
      strid, "", set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_int(
    const std::string &strid,
    const std::string &parent_strid,
    Property2<unsigned int>::set_cb_t set,
    Property2<unsigned int>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned int default_value,
    unsigned int min,
    unsigned int max){
  return make_under_parent<unsigned int>(
      strid, parent_strid, set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_short(
    const std::string &strid,
    Property2<unsigned short>::set_cb_t set,
    Property2<unsigned short>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned short default_value,
    unsigned short min,
    unsigned short max){
  return make_under_parent<unsigned short>(
      strid, "", set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_short(
    const std::string &strid,
    const std::string &parent_strid,
    Property2<unsigned short>::set_cb_t set,
    Property2<unsigned short>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned short default_value,
    unsigned short min,
    unsigned short max){
  return make_under_parent<unsigned short>(
      strid, parent_strid, set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_long(
    const std::string &strid,
    Property2<unsigned long>::set_cb_t set,
    Property2<unsigned long>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned long default_value,
    unsigned long min,
    unsigned long max){
  return make_under_parent<unsigned long>(
      strid, "", set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_long(
    const std::string &strid,
    const std::string &parent_strid,
    Property2<unsigned long>::set_cb_t set,
    Property2<unsigned long>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned long default_value,
    unsigned long min,
    unsigned long max){
  return make_under_parent<unsigned long>(
      strid, parent_strid, set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_long_long(
    const std::string &strid,
    Property2<unsigned long long>::set_cb_t set,
    Property2<unsigned long long>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned long long default_value,
    unsigned long long min,
    unsigned long long max){
  return make_under_parent<unsigned long long>(
      strid, "", set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_long_long(
    const std::string &strid,
    const std::string &parent_strid,
    Property2<unsigned long long>::set_cb_t set,
    Property2<unsigned long long>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    unsigned long long default_value,
    unsigned long long min,
    unsigned long long max){
  return make_under_parent<unsigned long long>(
      strid, parent_strid, set, get, label, description,
      default_value, min, max);
}


PContainer::prop_id_t PContainer::make_bool(const std::string &strid,
                                            Property2<bool>::set_cb_t set,
                                            Property2<bool>::get_cb_t get,
                                            const std::string &label,
                                            const std::string &description,
                                            bool default_value){
  return make_under_parent<bool>(strid, "", set, get, label, description,
                                 default_value);
}

PContainer::prop_id_t PContainer::make_parented_bool(const std::string &strid,
                                                     const std::string &parent_strid,
                                                     Property2<bool>::set_cb_t set,
                                                     Property2<bool>::get_cb_t get,
                                                     const std::string &label,
                                                     const std::string &description,
                                                     bool default_value){
  return make_under_parent<bool>(strid, parent_strid, set, get, label, description,
                                 default_value);
}

PContainer::prop_id_t PContainer::make_float(const std::string &strid,
                                             Property2<float>::set_cb_t set,
                                             Property2<float>::get_cb_t get,
                                             const std::string &label,
                                             const std::string &description,
                                             float default_value,
                                             float min,
                                             float max){
  return make_under_parent<float>(strid, "", set, get, label, description,
                                  default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_float(const std::string &strid,
                                                      const std::string &parent_strid,
                                                      Property2<float>::set_cb_t set,
                                                      Property2<float>::get_cb_t get,
                                                      const std::string &label,
                                                      const std::string &description,
                                                      float default_value,
                                                      float min,
                                                      float max){
  return make_under_parent<float>(strid, parent_strid, set, get, label, description,
                                  default_value, min, max);
}

PContainer::prop_id_t PContainer::make_double(const std::string &strid,
                                              Property2<double>::set_cb_t set,
                                              Property2<double>::get_cb_t get,
                                              const std::string &label,
                                              const std::string &description,
                                              double default_value,
                                              double min,
                                              double max){
  return make_under_parent<double>(strid, "", set, get, label, description,
                                   default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_double(const std::string &strid,
                                                       const std::string &parent_strid,
                                                       Property2<double>::set_cb_t set,
                                                       Property2<double>::get_cb_t get,
                                                       const std::string &label,
                                                       const std::string &description,
                                                       double default_value,
                                                       double min,
                                                       double max){
  return make_under_parent<double>(strid, parent_strid, set, get, label, description,
                                   default_value, min, max);
}

PContainer::prop_id_t PContainer::make_long_double(const std::string &strid,
                                              Property2<long double>::set_cb_t set,
                                              Property2<long double>::get_cb_t get,
                                              const std::string &label,
                                              const std::string &description,
                                              long double default_value,
                                              long double min,
                                              long double max){
  return make_under_parent<long double>(strid, "", set, get, label, description,
                                        default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_long_double(
    const std::string &strid,
    const std::string &parent_strid,
    Property2<long double>::set_cb_t set,
    Property2<long double>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    long double default_value,
    long double min,
    long double max){
  return make_under_parent<long double>(
      strid, parent_strid, set, get, label, description,
      default_value, min, max);
}

PContainer::prop_id_t PContainer::make_string(const std::string &strid,
                                              Property2<std::string>::set_cb_t set,
                                              Property2<std::string>::get_cb_t get,
                                              const std::string &label,
                                              const std::string &description,
                                              std::string default_value){
  return make_under_parent<std::string>(strid, "", set, get, label, description,
                                        default_value);
}

PContainer::prop_id_t PContainer::make_parented_string(const std::string &strid,
                                                       const std::string &parent_strid,
                                                       Property2<std::string>::set_cb_t set,
                                                       Property2<std::string>::get_cb_t get,
                                                       const std::string &label,
                                                       const std::string &description,
                                                       std::string default_value){
  return make_under_parent<std::string>(strid, parent_strid, set, get, label, description,
                                        default_value);
}

PContainer::prop_id_t PContainer::make_selection(
    const std::string &strid,
    Property2<Selection, size_t>::set_cb_t set,
    Property2<Selection, size_t>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    const Selection &default_value){
  return make_under_parent<Selection, Selection::index_t>(
      strid, "", set, get, label, description,
      default_value, default_value.size() - 1);
}

PContainer::prop_id_t PContainer::make_parented_selection(
    const std::string &strid,
    const std::string &parent_strid,
    Property2<Selection, size_t>::set_cb_t set,
    Property2<Selection, size_t>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    const Selection &default_value){
  return make_under_parent<Selection, Selection::index_t>(
      strid, parent_strid, set, get, label, description,
      default_value, default_value.size() - 1);
}

PContainer::prop_id_t PContainer::make_group(
    const std::string &strid,
    const std::string &label,
    const std::string &description){
  return make_under_parent<Group>(strid, "", label, description);
}

PContainer::prop_id_t PContainer::make_parented_group(
    const std::string &strid,
    const std::string &parent_strid,
    const std::string &label,
    const std::string &description){
  return make_under_parent<Group>(strid, parent_strid, label, description);
}

PContainer::prop_id_t PContainer::make_char(const std::string &strid,
                                            Property2<char>::set_cb_t set,
                                            Property2<char>::get_cb_t get,
                                            const std::string &label,
                                            const std::string &description,
                                            char default_value){
  return make_under_parent<char>(strid, "", set, get, label, description,
                                 default_value);
}

PContainer::prop_id_t PContainer::make_parented_char(const std::string &strid,
                                                     const std::string &parent_strid,
                                                     Property2<char>::set_cb_t set,
                                                     Property2<char>::get_cb_t get,
                                                     const std::string &label,
                                                     const std::string &description,
                                                     char default_value){
  return make_under_parent<char>(strid, parent_strid, set, get, label, description,
                                 default_value);
}

PContainer::prop_id_t PContainer::make_fraction(
    const std::string &strid,
    Property2<Fraction>::set_cb_t set,
    Property2<Fraction>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    const Fraction &default_value,
    Fraction::ator_t min_num,
    Fraction::ator_t min_denom,
    Fraction::ator_t max_num,
    Fraction::ator_t max_denom){
  return make_under_parent<Fraction>(strid, "", set, get, label, description,
                                     default_value,
                                     min_num, min_denom,
                                     max_num, max_denom);
}

PContainer::prop_id_t PContainer::make_parented_fraction(
    const std::string &strid,
    const std::string &parent_strid,
    Property2<Fraction>::set_cb_t set,
    Property2<Fraction>::get_cb_t get,
    const std::string &label,
    const std::string &description,
    const Fraction &default_value,
    Fraction::ator_t min_num,
    Fraction::ator_t min_denom,
    Fraction::ator_t max_num,
    Fraction::ator_t max_denom){
  return make_under_parent<Fraction>(strid, parent_strid, set, get, label, description,
                                     default_value,
                                     min_num, min_denom,
                                     max_num, max_denom);
}

}  // namespace switcher
