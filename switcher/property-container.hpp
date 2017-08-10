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

#include <assert.h>
#include <glib.h>  // logs
#include <map>
#include <memory>
#include <string>
#include "./counter-map.hpp"
#include "./is-specialization-of.hpp"
#include "./property-internal-types.hpp"
#include "./property.hpp"

namespace switcher {
class PContainer {
  friend class Bundle;  // replacing some methods like replace and delete
 public:
  using prop_id_t = PropertyBase::prop_id_t;
  using notify_cb_t = PropertyBase::notify_cb_t;
  using register_id_t = PropertyBase::register_id_t;
  using on_tree_grafted_cb_t = std::function<void(const std::string& key)>;
  using on_tree_pruned_cb_t = std::function<void(const std::string& key)>;
  PContainer() = delete;
  // ctor will own tree and write into .property.
  PContainer(InfoTree::ptr tree,
             on_tree_grafted_cb_t on_tree_grafted_cb,
             on_tree_pruned_cb_t on_tree_pruned_cb);

  // ------------- use (const methods)
  // return 0 if id is not found
  prop_id_t get_id(const std::string& id) const;
  std::string get_name(prop_id_t id) const;
  std::vector<std::pair<std::string, PContainer::prop_id_t>> get_ids() const;
  std::map<prop_id_t, std::string> get_names() const;

  register_id_t subscribe(prop_id_t id, notify_cb_t fun) const;
  bool unsubscribe(prop_id_t id, register_id_t rid) const;

  bool set_str(prop_id_t id, const std::string& val) const;
  std::string get_str(prop_id_t id) const;
  bool set_str_str(const std::string& strid, const std::string& val) const;
  std::string get_str_str(const std::string& strid) const;
  template <typename T>
  bool set(prop_id_t id, const T& val) const {
    auto prop_it = props_.find(id);
#ifdef DEBUG
    assert(prop_it->second->get()->get_type_id_hash() == typeid(val).hash_code());
#endif
    return static_cast<Property<T>*>(prop_it->second->get())->set(std::forward<const T&>(val));
  }
  template <typename T>
  T get(prop_id_t id) const {
    const auto& prop_it = props_.find(id);
#ifdef DEBUG
    assert(prop_it->second->get()->get_type_id_hash() != typeid(T).hash_code());
#endif
    return static_cast<Property<T>*>(prop_it->second->get())->get();
  }

  // ----------- add/remove/update (you should prefer makers for adding)

  bool remove(prop_id_t prop_id);
  bool enable(prop_id_t prop_id);
  bool disable(prop_id_t prop_id, const std::string& why);
  prop_id_t push(const std::string& strid, std::unique_ptr<PropertyBase>&& prop_ptr);
  prop_id_t push_parented(const std::string& strid,
                          const std::string& parent_strid,
                          std::unique_ptr<PropertyBase>&& prop_ptr);
  bool replace(prop_id_t prop_id, std::unique_ptr<PropertyBase>&& prop_ptr) {
    return replace_impl(prop_id, std::forward<std::unique_ptr<PropertyBase>>(prop_ptr), false);
  }
  bool replace_and_notify(prop_id_t prop_id, std::unique_ptr<PropertyBase>&& prop_ptr) {
    return replace_impl(prop_id, std::forward<std::unique_ptr<PropertyBase>>(prop_ptr), true);
  }

  // use when property is updated without "set" (method is read-only for
  // instance)
  std::unique_lock<std::mutex> get_lock(prop_id_t prop_id);
  void notify(prop_id_t prop_id);
  void set_to_current(prop_id_t id);
  void update_value_in_tree(prop_id_t prop) const;
  void update_values_in_tree() const;

  // ----------- makers
  prop_id_t make_int(const std::string& strid,
                     prop::set_t<int> set,
                     prop::get_t<int> get,
                     const std::string& label,
                     const std::string& description,
                     int default_value,
                     int min,
                     int max);

  prop_id_t make_parented_int(const std::string& strid,
                              const std::string& parent_strid,
                              prop::set_t<int> set,
                              prop::get_t<int> get,
                              const std::string& label,
                              const std::string& description,
                              int default_value,
                              int min,
                              int max);

  prop_id_t make_short(const std::string& strid,
                       prop::set_t<short> set,
                       prop::get_t<short> get,
                       const std::string& label,
                       const std::string& description,
                       short default_value,
                       short min,
                       short max);

  prop_id_t make_parented_short(const std::string& strid,
                                const std::string& parent_strid,
                                prop::set_t<short> set,
                                prop::get_t<short> get,
                                const std::string& label,
                                const std::string& description,
                                short default_value,
                                short min,
                                short max);

  prop_id_t make_long(const std::string& strid,
                      prop::set_t<long> set,
                      prop::get_t<long> get,
                      const std::string& label,
                      const std::string& description,
                      long default_value,
                      long min,
                      long max);

  prop_id_t make_parented_long(const std::string& strid,
                               const std::string& parent_strid,
                               prop::set_t<long> set,
                               prop::get_t<long> get,
                               const std::string& label,
                               const std::string& description,
                               long default_value,
                               long min,
                               long max);

  prop_id_t make_long_long(const std::string& strid,
                           Property<long long>::set_cb_t set,
                           Property<long long>::get_cb_t get,
                           const std::string& label,
                           const std::string& description,
                           long long default_value,
                           long long min,
                           long long max);

  prop_id_t make_parented_long_long(const std::string& strid,
                                    const std::string& parent_strid,
                                    Property<long long>::set_cb_t set,
                                    Property<long long>::get_cb_t get,
                                    const std::string& label,
                                    const std::string& description,
                                    long long default_value,
                                    long long min,
                                    long long max);

  prop_id_t make_unsigned_int(const std::string& strid,
                              Property<unsigned int>::set_cb_t set,
                              Property<unsigned int>::get_cb_t get,
                              const std::string& label,
                              const std::string& description,
                              unsigned int default_value,
                              unsigned int min,
                              unsigned int max);

  prop_id_t make_parented_unsigned_int(const std::string& strid,
                                       const std::string& parent_strid,
                                       Property<unsigned int>::set_cb_t set,
                                       Property<unsigned int>::get_cb_t get,
                                       const std::string& label,
                                       const std::string& description,
                                       unsigned int default_value,
                                       unsigned int min,
                                       unsigned int max);

  prop_id_t make_unsigned_short(const std::string& strid,
                                Property<unsigned short>::set_cb_t set,
                                Property<unsigned short>::get_cb_t get,
                                const std::string& label,
                                const std::string& description,
                                unsigned short default_value,
                                unsigned short min,
                                unsigned short max);

  prop_id_t make_parented_unsigned_short(const std::string& strid,
                                         const std::string& parent_strid,
                                         Property<unsigned short>::set_cb_t set,
                                         Property<unsigned short>::get_cb_t get,
                                         const std::string& label,
                                         const std::string& description,
                                         unsigned short default_value,
                                         unsigned short min,
                                         unsigned short max);

  prop_id_t make_unsigned_long(const std::string& strid,
                               Property<unsigned long>::set_cb_t set,
                               Property<unsigned long>::get_cb_t get,
                               const std::string& label,
                               const std::string& description,
                               unsigned long default_value,
                               unsigned long min,
                               unsigned long max);

  prop_id_t make_parented_unsigned_long(const std::string& strid,
                                        const std::string& parent_strid,
                                        Property<unsigned long>::set_cb_t set,
                                        Property<unsigned long>::get_cb_t get,
                                        const std::string& label,
                                        const std::string& description,
                                        unsigned long default_value,
                                        unsigned long min,
                                        unsigned long max);

  prop_id_t make_unsigned_long_long(const std::string& strid,
                                    Property<unsigned long long>::set_cb_t set,
                                    Property<unsigned long long>::get_cb_t get,
                                    const std::string& label,
                                    const std::string& description,
                                    unsigned long long default_value,
                                    unsigned long long min,
                                    unsigned long long max);

  prop_id_t make_parented_unsigned_long_long(const std::string& strid,
                                             const std::string& parent_strid,
                                             Property<unsigned long long>::set_cb_t set,
                                             Property<unsigned long long>::get_cb_t get,
                                             const std::string& label,
                                             const std::string& description,
                                             unsigned long long default_value,
                                             unsigned long long min,
                                             unsigned long long max);

  prop_id_t make_bool(const std::string& strid,
                      prop::set_t<bool> set,
                      prop::get_t<bool> get,
                      const std::string& label,
                      const std::string& description,
                      bool default_value);

  prop_id_t make_parented_bool(const std::string& strid,
                               const std::string& parent_strid,
                               prop::set_t<bool> set,
                               prop::get_t<bool> get,
                               const std::string& label,
                               const std::string& description,
                               bool default_value);

  prop_id_t make_float(const std::string& strid,
                       prop::set_t<float> set,
                       prop::get_t<float> get,
                       const std::string& label,
                       const std::string& description,
                       float default_value,
                       float min,
                       float max);

  prop_id_t make_parented_float(const std::string& strid,
                                const std::string& parent_strid,
                                prop::set_t<float> set,
                                prop::get_t<float> get,
                                const std::string& label,
                                const std::string& description,
                                float default_value,
                                float min,
                                float max);

  prop_id_t make_double(const std::string& strid,
                        prop::set_t<double> set,
                        prop::get_t<double> get,
                        const std::string& label,
                        const std::string& description,
                        double default_value,
                        double min,
                        double max);

  prop_id_t make_parented_double(const std::string& strid,
                                 const std::string& parent_strid,
                                 prop::set_t<double> set,
                                 prop::get_t<double> get,
                                 const std::string& label,
                                 const std::string& description,
                                 double default_value,
                                 double min,
                                 double max);

  prop_id_t make_long_double(const std::string& strid,
                             Property<long double>::set_cb_t set,
                             Property<long double>::get_cb_t get,
                             const std::string& label,
                             const std::string& description,
                             long double default_value,
                             long double min,
                             long double max);

  prop_id_t make_parented_long_double(const std::string& strid,
                                      const std::string& parent_strid,
                                      Property<long double>::set_cb_t set,
                                      Property<long double>::get_cb_t get,
                                      const std::string& label,
                                      const std::string& description,
                                      long double default_value,
                                      long double min,
                                      long double max);

  prop_id_t make_char(const std::string& strid,
                      prop::set_t<char> set,
                      prop::get_t<char> get,
                      const std::string& label,
                      const std::string& description,
                      char default_value);

  prop_id_t make_parented_char(const std::string& strid,
                               const std::string& parent_strid,
                               prop::set_t<char> set,
                               prop::get_t<char> get,
                               const std::string& label,
                               const std::string& description,
                               char default_value);

  prop_id_t make_string(const std::string& strid,
                        Property<std::string>::set_cb_t set,
                        Property<std::string>::get_cb_t get,
                        const std::string& label,
                        const std::string& description,
                        std::string default_value);

  prop_id_t make_parented_string(const std::string& strid,
                                 const std::string& parent_strid,
                                 Property<std::string>::set_cb_t set,
                                 Property<std::string>::get_cb_t get,
                                 const std::string& label,
                                 const std::string& description,
                                 std::string default_value);

  template <typename T = std::string>
  prop_id_t make_selection(const std::string& strid,
                           std::function<bool(const IndexOrName&)> set,
                           std::function<IndexOrName()> get,
                           const std::string& label,
                           const std::string& description,
                           const Selection<T>& default_value) {
    return make_under_parent<Selection<T>, IndexOrName>(
        strid, "", set, get, label, description, default_value, default_value.size() - 1);
  }

  template <typename T = std::string>
  prop_id_t make_parented_selection(const std::string& strid,
                                    const std::string& parent_strid,
                                    std::function<bool(const IndexOrName&)> set,
                                    std::function<IndexOrName()> get,
                                    const std::string& label,
                                    const std::string& description,
                                    const Selection<T>& default_value) {
    return make_under_parent<Selection<T>, IndexOrName>(
        strid, parent_strid, set, get, label, description, default_value, default_value.size() - 1);
  }

  prop_id_t make_group(const std::string& strid,
                       const std::string& label,
                       const std::string& description);

  prop_id_t make_parented_group(const std::string& strid,
                                const std::string& parent_strid,
                                const std::string& label,
                                const std::string& description);

  prop_id_t make_fraction(const std::string& strid,
                          prop::set_t<Fraction> set,
                          prop::get_t<Fraction> get,
                          const std::string& label,
                          const std::string& description,
                          const Fraction& default_value,
                          Fraction::ator_t min_num,
                          Fraction::ator_t min_denom,
                          Fraction::ator_t max_num,
                          Fraction::ator_t max_denom);

  prop_id_t make_parented_fraction(const std::string& strid,
                                   const std::string& parent_strid,
                                   prop::set_t<Fraction> set,
                                   prop::get_t<Fraction> get,
                                   const std::string& label,
                                   const std::string& description,
                                   const Fraction& default_value,
                                   Fraction::ator_t min_num,
                                   Fraction::ator_t min_denom,
                                   Fraction::ator_t max_num,
                                   Fraction::ator_t max_denom);

  prop_id_t make_color(const std::string& strid,
                       prop::set_t<Color> set,
                       prop::get_t<Color> get,
                       const std::string& label,
                       const std::string& description,
                       const Color& default_value);

  prop_id_t make_parented_color(const std::string& strid,
                                const std::string& parent_strid,
                                prop::set_t<Color> set,
                                prop::get_t<Color> get,
                                const std::string& label,
                                const std::string& description,
                                const Color& default_value);

  // forward_property mirrors properties from the property container in the current container
  prop_id_t mirror_property_from(const std::string& strid,
                                 const std::string& parent_strid,
                                 PContainer* pc,
                                 prop_id_t prop_id);

  template <typename T>
  prop_id_t make_tuple(const std::string& strid,
                       std::function<bool(const T&)> set,
                       std::function<T()> get,
                       const std::string& label,
                       const std::string& description,
                       const T& default_value) {
    static_assert(is_specialization_of<std::tuple, T>::value,
                  "make_tuple requires a std::tuple as template parameter,"
                  " something else was given");
    return make_under_parent<T>(
        strid, "", set, get, label, description, std::forward<const T&>(default_value));
  }

  template <typename... T>
  prop_id_t make_parented_tuple(const std::string& strid,
                                const std::string& parent_strid,
                                std::function<bool(const std::tuple<T...>&)> set,
                                std::function<std::tuple<T...>()> get,
                                const std::string& label,
                                const std::string& description,
                                const std::tuple<T...>& default_value) {
    return make_under_parent<std::tuple<T...>>(
        strid,
        parent_strid,
        set,
        get,
        label,
        description,
        std::forward<const std::tuple<T...>&>(default_value));
  }

 private:
  prop_id_t counter_{0};
  // props_ and actual_props_ are maintained together: props_ is an indirection for methods using
  // properties, and actual_props_ is used for property ownership.
  // This allows bundle quiddities to mirror actual_props_ using their own property ids.
  std::map<prop_id_t, std::unique_ptr<PropertyBase>*> props_{};
  std::map<prop_id_t, std::unique_ptr<PropertyBase>> actual_props_{};
  std::map<std::string, prop_id_t> ids_{};
  std::map<prop_id_t, std::string> strids_{};
  InfoTree::ptr tree_;
  on_tree_grafted_cb_t on_tree_grafted_cb_;
  on_tree_pruned_cb_t on_tree_pruned_cb_;
  CounterMap suborders_{};

  template <typename PropType, typename PropGetSet = PropType, typename... PropArgs>
  prop_id_t make_under_parent(const std::string& strid,
                              const std::string& parent_strid,
                              PropArgs... args) {
    return push_parented(
        strid,
        parent_strid,
        std::make_unique<Property<PropType, PropGetSet>>(std::forward<PropArgs>(args)...));
  }
  void init_newly_installed_property(const std::string& strid,
                                     const std::string& parent_strid,
                                     size_t pos);

  bool replace_impl(prop_id_t prop_id, std::unique_ptr<PropertyBase>&& prop_ptr, bool force_notify);
};

}  // namespace switcher
#endif
