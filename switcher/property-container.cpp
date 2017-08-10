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

PContainer::PContainer(InfoTree::ptr tree,
                       on_tree_grafted_cb_t on_tree_grafted_cb,
                       on_tree_pruned_cb_t on_tree_pruned_cb)
    : tree_(tree), on_tree_grafted_cb_(on_tree_grafted_cb), on_tree_pruned_cb_(on_tree_pruned_cb) {
  tree_->graft(".property", InfoTree::make());
  tree_->tag_as_array(".property", true);
}

bool PContainer::replace_impl(prop_id_t prop_id,
                              std::unique_ptr<PropertyBase>&& prop_ptr,
                              bool force_notify) {
  auto it = strids_.find(prop_id);
  auto strid = it->second;
  if (strids_.end() == it) return false;  // prop not found
  auto old_value = get_str(prop_id);
  // keep a reference to the old property documentation tree
  auto* prop = props_[prop_id]->get();
  auto old_tree = prop->get_spec();
  // copy notification cbs
  auto notification_cbs = prop->get_notify_cbs();
  // replace with new prop
  actual_props_[prop_id] = std::forward<std::unique_ptr<PropertyBase>>(prop_ptr);
  props_[prop_id] = &actual_props_[prop_id];
  prop = props_[prop_id]->get();
  prop->set_notify_cbs(notification_cbs);
  prop->set_id(prop_id);
  prop->set_str(old_value);
  // place old tree into new property
  auto key = std::string("property.") + strid;
  auto tree = prop->get_spec();
  tree->graft("id", InfoTree::make(old_tree->branch_get_value("id")));
  tree->graft("prop_id", InfoTree::make(old_tree->branch_get_value("prop_id")));
  tree->graft("order", InfoTree::make(old_tree->branch_get_value("order")));
  tree->graft("parent", InfoTree::make(old_tree->branch_get_value("parent")));
  tree->graft("enabled", InfoTree::make(old_tree->branch_get_value("enabled")));

  // updating tree_
  tree_->graft(key, tree);
  if (force_notify && on_tree_grafted_cb_) on_tree_grafted_cb_(key);
  return true;
}

bool PContainer::remove(prop_id_t prop_id) {
  auto it = strids_.find(prop_id);
  if (strids_.end() == it) return false;  // prop not found
  auto key = std::string("property.") + it->second;
  tree_->prune(key);
  if (on_tree_pruned_cb_) on_tree_pruned_cb_(key);
  ids_.erase(it->second);
  strids_.erase(it);
  props_.erase(prop_id);
  actual_props_.erase(prop_id);
  return true;
}

bool PContainer::enable(prop_id_t prop_id) {
  const auto& it = strids_.find(prop_id);
  if (strids_.end() == it) return false;
  auto key = std::string("property.") + it->second + ".enabled";
  tree_->graft(key, InfoTree::make(true));
  auto why_key = std::string("property.") + it->second + ".why_disabled";
  tree_->graft(why_key, InfoTree::make(""));
  if (on_tree_grafted_cb_) on_tree_grafted_cb_(key);
  return true;
}

bool PContainer::disable(prop_id_t prop_id, const std::string& why) {
  const auto& it = strids_.find(prop_id);
  if (strids_.end() == it) return false;
  auto key = std::string("property.") + it->second + ".enabled";
  tree_->graft(key, InfoTree::make(false));
  auto why_key = std::string("property.") + it->second + ".why_disabled";
  tree_->graft(why_key, InfoTree::make(why));
  if (on_tree_grafted_cb_) on_tree_grafted_cb_(key);
  return true;
}

PContainer::register_id_t PContainer::subscribe(prop_id_t id, notify_cb_t fun) const {
  auto prop = props_.find(id);
  if (prop == props_.end()) return 0;
  return prop->second->get()->subscribe(std::forward<notify_cb_t>(fun));
}

bool PContainer::unsubscribe(prop_id_t id, register_id_t rid) const {
  auto prop = props_.find(id);
  if (prop == props_.end()) return false;
  return prop->second->get()->unsubscribe(std::forward<register_id_t>(rid));
}

PContainer::prop_id_t PContainer::get_id(const std::string& strid) const {
  const auto& it = ids_.find(strid);
  if (ids_.end() != it) return it->second;
  // accepting id converted to string
  auto id = prop::id_from_string(strid);
  if (props_.end() != props_.find(id)) return id;
  return 0;
}

std::vector<std::pair<std::string, PContainer::prop_id_t>> PContainer::get_ids() const {
  std::vector<std::pair<std::string, PContainer::prop_id_t>> ids;
  for (auto& prop : ids_) {
    ids.push_back(std::make_pair(prop.first, prop.second));
  }
  return ids;
}

std::string PContainer::get_name(prop_id_t id) const {
  const auto& it = strids_.find(id);
  if (strids_.end() != it) return it->second;
  return std::string();
}

std::map<PContainer::prop_id_t, std::string> PContainer::get_names() const { return strids_; }

PContainer::prop_id_t PContainer::make_int(const std::string& strid,
                                           Property<int>::set_cb_t set,
                                           Property<int>::get_cb_t get,
                                           const std::string& label,
                                           const std::string& description,
                                           int default_value,
                                           int min,
                                           int max) {
  return make_under_parent<int>(strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_int(const std::string& strid,
                                                    const std::string& parent_strid,
                                                    Property<int>::set_cb_t set,
                                                    Property<int>::get_cb_t get,
                                                    const std::string& label,
                                                    const std::string& description,
                                                    int default_value,
                                                    int min,
                                                    int max) {
  return make_under_parent<int>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_short(const std::string& strid,
                                             Property<short>::set_cb_t set,
                                             Property<short>::get_cb_t get,
                                             const std::string& label,
                                             const std::string& description,
                                             short default_value,
                                             short min,
                                             short max) {
  return make_under_parent<short>(strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_short(const std::string& strid,
                                                      const std::string& parent_strid,
                                                      Property<short>::set_cb_t set,
                                                      Property<short>::get_cb_t get,
                                                      const std::string& label,
                                                      const std::string& description,
                                                      short default_value,
                                                      short min,
                                                      short max) {
  return make_under_parent<short>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_long(const std::string& strid,
                                            Property<long>::set_cb_t set,
                                            Property<long>::get_cb_t get,
                                            const std::string& label,
                                            const std::string& description,
                                            long default_value,
                                            long min,
                                            long max) {
  return make_under_parent<long>(strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_long(const std::string& strid,
                                                     const std::string& parent_strid,
                                                     Property<long>::set_cb_t set,
                                                     Property<long>::get_cb_t get,
                                                     const std::string& label,
                                                     const std::string& description,
                                                     long default_value,
                                                     long min,
                                                     long max) {
  return make_under_parent<long>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_long_long(const std::string& strid,
                                                 Property<long long>::set_cb_t set,
                                                 Property<long long>::get_cb_t get,
                                                 const std::string& label,
                                                 const std::string& description,
                                                 long long default_value,
                                                 long long min,
                                                 long long max) {
  return make_under_parent<long long>(
      strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_long_long(const std::string& strid,
                                                          const std::string& parent_strid,
                                                          Property<long long>::set_cb_t set,
                                                          Property<long long>::get_cb_t get,
                                                          const std::string& label,
                                                          const std::string& description,
                                                          long long default_value,
                                                          long long min,
                                                          long long max) {
  return make_under_parent<long long>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_int(const std::string& strid,
                                                    Property<unsigned int>::set_cb_t set,
                                                    Property<unsigned int>::get_cb_t get,
                                                    const std::string& label,
                                                    const std::string& description,
                                                    unsigned int default_value,
                                                    unsigned int min,
                                                    unsigned int max) {
  return make_under_parent<unsigned int>(
      strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_int(const std::string& strid,
                                                             const std::string& parent_strid,
                                                             Property<unsigned int>::set_cb_t set,
                                                             Property<unsigned int>::get_cb_t get,
                                                             const std::string& label,
                                                             const std::string& description,
                                                             unsigned int default_value,
                                                             unsigned int min,
                                                             unsigned int max) {
  return make_under_parent<unsigned int>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_short(const std::string& strid,
                                                      Property<unsigned short>::set_cb_t set,
                                                      Property<unsigned short>::get_cb_t get,
                                                      const std::string& label,
                                                      const std::string& description,
                                                      unsigned short default_value,
                                                      unsigned short min,
                                                      unsigned short max) {
  return make_under_parent<unsigned short>(
      strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_short(
    const std::string& strid,
    const std::string& parent_strid,
    Property<unsigned short>::set_cb_t set,
    Property<unsigned short>::get_cb_t get,
    const std::string& label,
    const std::string& description,
    unsigned short default_value,
    unsigned short min,
    unsigned short max) {
  return make_under_parent<unsigned short>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_long(const std::string& strid,
                                                     Property<unsigned long>::set_cb_t set,
                                                     Property<unsigned long>::get_cb_t get,
                                                     const std::string& label,
                                                     const std::string& description,
                                                     unsigned long default_value,
                                                     unsigned long min,
                                                     unsigned long max) {
  return make_under_parent<unsigned long>(
      strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_long(const std::string& strid,
                                                              const std::string& parent_strid,
                                                              Property<unsigned long>::set_cb_t set,
                                                              Property<unsigned long>::get_cb_t get,
                                                              const std::string& label,
                                                              const std::string& description,
                                                              unsigned long default_value,
                                                              unsigned long min,
                                                              unsigned long max) {
  return make_under_parent<unsigned long>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_unsigned_long_long(
    const std::string& strid,
    Property<unsigned long long>::set_cb_t set,
    Property<unsigned long long>::get_cb_t get,
    const std::string& label,
    const std::string& description,
    unsigned long long default_value,
    unsigned long long min,
    unsigned long long max) {
  return make_under_parent<unsigned long long>(
      strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_unsigned_long_long(
    const std::string& strid,
    const std::string& parent_strid,
    Property<unsigned long long>::set_cb_t set,
    Property<unsigned long long>::get_cb_t get,
    const std::string& label,
    const std::string& description,
    unsigned long long default_value,
    unsigned long long min,
    unsigned long long max) {
  return make_under_parent<unsigned long long>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_bool(const std::string& strid,
                                            Property<bool>::set_cb_t set,
                                            Property<bool>::get_cb_t get,
                                            const std::string& label,
                                            const std::string& description,
                                            bool default_value) {
  return make_under_parent<bool>(strid, "", set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::make_parented_bool(const std::string& strid,
                                                     const std::string& parent_strid,
                                                     Property<bool>::set_cb_t set,
                                                     Property<bool>::get_cb_t get,
                                                     const std::string& label,
                                                     const std::string& description,
                                                     bool default_value) {
  return make_under_parent<bool>(strid, parent_strid, set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::make_float(const std::string& strid,
                                             Property<float>::set_cb_t set,
                                             Property<float>::get_cb_t get,
                                             const std::string& label,
                                             const std::string& description,
                                             float default_value,
                                             float min,
                                             float max) {
  return make_under_parent<float>(strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_float(const std::string& strid,
                                                      const std::string& parent_strid,
                                                      Property<float>::set_cb_t set,
                                                      Property<float>::get_cb_t get,
                                                      const std::string& label,
                                                      const std::string& description,
                                                      float default_value,
                                                      float min,
                                                      float max) {
  return make_under_parent<float>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_double(const std::string& strid,
                                              Property<double>::set_cb_t set,
                                              Property<double>::get_cb_t get,
                                              const std::string& label,
                                              const std::string& description,
                                              double default_value,
                                              double min,
                                              double max) {
  return make_under_parent<double>(
      strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_double(const std::string& strid,
                                                       const std::string& parent_strid,
                                                       Property<double>::set_cb_t set,
                                                       Property<double>::get_cb_t get,
                                                       const std::string& label,
                                                       const std::string& description,
                                                       double default_value,
                                                       double min,
                                                       double max) {
  return make_under_parent<double>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_long_double(const std::string& strid,
                                                   Property<long double>::set_cb_t set,
                                                   Property<long double>::get_cb_t get,
                                                   const std::string& label,
                                                   const std::string& description,
                                                   long double default_value,
                                                   long double min,
                                                   long double max) {
  return make_under_parent<long double>(
      strid, "", set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_parented_long_double(const std::string& strid,
                                                            const std::string& parent_strid,
                                                            Property<long double>::set_cb_t set,
                                                            Property<long double>::get_cb_t get,
                                                            const std::string& label,
                                                            const std::string& description,
                                                            long double default_value,
                                                            long double min,
                                                            long double max) {
  return make_under_parent<long double>(
      strid, parent_strid, set, get, label, description, default_value, min, max);
}

PContainer::prop_id_t PContainer::make_string(const std::string& strid,
                                              Property<std::string>::set_cb_t set,
                                              Property<std::string>::get_cb_t get,
                                              const std::string& label,
                                              const std::string& description,
                                              std::string default_value) {
  return make_under_parent<std::string>(strid, "", set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::make_parented_string(const std::string& strid,
                                                       const std::string& parent_strid,
                                                       Property<std::string>::set_cb_t set,
                                                       Property<std::string>::get_cb_t get,
                                                       const std::string& label,
                                                       const std::string& description,
                                                       std::string default_value) {
  return make_under_parent<std::string>(
      strid, parent_strid, set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::make_group(const std::string& strid,
                                             const std::string& label,
                                             const std::string& description) {
  return make_under_parent<Group>(strid, "", label, description);
}

PContainer::prop_id_t PContainer::make_parented_group(const std::string& strid,
                                                      const std::string& parent_strid,
                                                      const std::string& label,
                                                      const std::string& description) {
  return make_under_parent<Group>(strid, parent_strid, label, description);
}

PContainer::prop_id_t PContainer::make_char(const std::string& strid,
                                            Property<char>::set_cb_t set,
                                            Property<char>::get_cb_t get,
                                            const std::string& label,
                                            const std::string& description,
                                            char default_value) {
  return make_under_parent<char>(strid, "", set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::make_parented_char(const std::string& strid,
                                                     const std::string& parent_strid,
                                                     Property<char>::set_cb_t set,
                                                     Property<char>::get_cb_t get,
                                                     const std::string& label,
                                                     const std::string& description,
                                                     char default_value) {
  return make_under_parent<char>(strid, parent_strid, set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::make_fraction(const std::string& strid,
                                                Property<Fraction>::set_cb_t set,
                                                Property<Fraction>::get_cb_t get,
                                                const std::string& label,
                                                const std::string& description,
                                                const Fraction& default_value,
                                                Fraction::ator_t min_num,
                                                Fraction::ator_t min_denom,
                                                Fraction::ator_t max_num,
                                                Fraction::ator_t max_denom) {
  return make_under_parent<Fraction>(strid,
                                     "",
                                     set,
                                     get,
                                     label,
                                     description,
                                     default_value,
                                     min_num,
                                     min_denom,
                                     max_num,
                                     max_denom);
}

PContainer::prop_id_t PContainer::make_parented_fraction(const std::string& strid,
                                                         const std::string& parent_strid,
                                                         Property<Fraction>::set_cb_t set,
                                                         Property<Fraction>::get_cb_t get,
                                                         const std::string& label,
                                                         const std::string& description,
                                                         const Fraction& default_value,
                                                         Fraction::ator_t min_num,
                                                         Fraction::ator_t min_denom,
                                                         Fraction::ator_t max_num,
                                                         Fraction::ator_t max_denom) {
  return make_under_parent<Fraction>(strid,
                                     parent_strid,
                                     set,
                                     get,
                                     label,
                                     description,
                                     default_value,
                                     min_num,
                                     min_denom,
                                     max_num,
                                     max_denom);
}

bool PContainer::set_str(prop_id_t id, const std::string& val) const {
  if (0 == id) return false;
  auto prop_it = props_.find(id);
  return prop_it->second->get()->set_str(std::forward<const std::string&>(val));
}

std::string PContainer::get_str(prop_id_t id) const {
  auto prop_it = props_.find(id);
  return prop_it->second->get()->get_str();
}

bool PContainer::set_str_str(const std::string& strid, const std::string& val) const {
  auto id = get_id(strid);
  if (0 != id) {
    auto prop_it = props_.find(id);
    return prop_it->second->get()->set_str(std::forward<const std::string&>(val));
  }
  // accepting id converted to string
  auto prop_id = prop::id_from_string(strid);
  auto prop_it = props_.find(prop_id);
  if (props_.end() == prop_it) return false;
  return prop_it->second->get()->set_str(std::forward<const std::string&>(val));
}

std::string PContainer::get_str_str(const std::string& strid) const {
  auto id = get_id(strid);
  if (0 != id) {
    auto prop_it = props_.find(id);
    return prop_it->second->get()->get_str();
  }
  // accepting id converted to string
  auto prop_id = prop::id_from_string(strid);
  auto prop_it = props_.find(prop_id);
  if (props_.end() == prop_it) return std::string();
  return prop_it->second->get()->get_str();
}

PContainer::prop_id_t PContainer::push(const std::string& strid,
                                       std::unique_ptr<PropertyBase>&& prop_ptr) {
  return push_parented(strid, "", std::forward<std::unique_ptr<PropertyBase>>(prop_ptr));
}

PContainer::prop_id_t PContainer::push_parented(const std::string& strid,
                                                const std::string& parent_strid,
                                                std::unique_ptr<PropertyBase>&& prop_ptr) {
  if (ids_.cend() != ids_.find(strid)) {
    return 0;
  }
  if (parent_strid != "" && ids_.cend() == ids_.find(parent_strid)) {
    return 0;
  }
  if (!prop_ptr) {
    return 0;
  }
  actual_props_[++counter_] = std::forward<std::unique_ptr<PropertyBase>>(prop_ptr);
  props_[counter_] = &actual_props_[counter_];
  init_newly_installed_property(strid, parent_strid, 20 * (suborders_.get_count(parent_strid) + 1));
  return counter_;
}

void PContainer::init_newly_installed_property(const std::string& strid,
                                               const std::string& parent_strid,
                                               size_t pos) {
  ids_[strid] = counter_;
  strids_[counter_] = strid;
  auto* prop = props_[counter_]->get();
  prop->set_id(counter_);
  auto tree = prop->get_spec();
  auto key = std::string("property.") + strid;
  tree_->graft(key, tree);
  tree->graft("id", InfoTree::make(strid));
  tree->graft("prop_id", InfoTree::make(counter_));
  tree->graft("order", InfoTree::make(pos));
  tree->graft("parent", InfoTree::make(parent_strid));
  tree->graft("enabled", InfoTree::make(true));
  if (on_tree_grafted_cb_) on_tree_grafted_cb_(key);
}

std::unique_lock<std::mutex> PContainer::get_lock(prop_id_t id) {
  return props_.find(id)->second->get()->get_lock();
}

void PContainer::notify(prop_id_t id) { props_.find(id)->second->get()->notify(); }

void PContainer::set_to_current(prop_id_t id) { props_.find(id)->second->get()->set_to_current(); }

void PContainer::update_values_in_tree() const {
  for (auto& it : props_) it.second->get()->update_value_in_spec();
}

void PContainer::update_value_in_tree(prop_id_t id) const {
  props_.find(id)->second->get()->update_value_in_spec();
}

PContainer::prop_id_t PContainer::make_color(const std::string& strid,
                                             Property<Color>::set_cb_t set,
                                             Property<Color>::get_cb_t get,
                                             const std::string& label,
                                             const std::string& description,
                                             const Color& default_value) {
  return make_under_parent<Color>(strid, "", set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::make_parented_color(const std::string& strid,
                                                      const std::string& parent_strid,
                                                      Property<Color>::set_cb_t set,
                                                      Property<Color>::get_cb_t get,
                                                      const std::string& label,
                                                      const std::string& description,
                                                      const Color& default_value) {
  return make_under_parent<Color>(strid, parent_strid, set, get, label, description, default_value);
}

PContainer::prop_id_t PContainer::mirror_property_from(const std::string& strid,
                                                       const std::string& parent_strid,
                                                       PContainer* pc,
                                                       prop_id_t prop_id) {
  if (ids_.cend() != ids_.find(strid)) return 0;  // strid already taken
  auto orig_parent =
      pc->actual_props_[prop_id]->get_spec()->branch_get_value(".parent").copy_as<std::string>();
  std::string new_parent_strid =
      orig_parent.empty() ? parent_strid
                          : (parent_strid.empty() ? orig_parent : parent_strid + "/" + orig_parent);
  if (new_parent_strid != "" && ids_.cend() == ids_.find(new_parent_strid))
    return 0;  // parent not found
  props_[++counter_] = &pc->actual_props_[prop_id];
  // maintain original order
  size_t pos = pc->actual_props_[prop_id]->get_spec()->branch_get_value(".order").copy_as<size_t>();
  init_newly_installed_property(strid, new_parent_strid, pos);
  return counter_;
}

}  // namespace switcher
