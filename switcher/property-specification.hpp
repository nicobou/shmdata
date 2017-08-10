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

#ifndef __SWITCHER_PROPERTY_SPECIFICATION_H__
#define __SWITCHER_PROPERTY_SPECIFICATION_H__

#include <glib.h>  //log

#include <functional>
#include <sstream>
#include <string>
#include <typeinfo>

#include "./color.hpp"
#include "./fraction.hpp"
#include "./group.hpp"
#include "./information-tree.hpp"
#include "./is-specialization-of.hpp"
#include "./selection.hpp"
#include "./templated-sequence.hpp"
#include "./type-name-registry.hpp"

namespace switcher {

template <typename T, typename TT = T>
class PropertySpecification {
 public:
  PropertySpecification() = delete;

  // typename V is here in order to allow value to be initialized from an other
  // type,
  // e.g. an unsigned int with an int

  template <typename U = T, typename V>
  PropertySpecification(
      bool is_writable,
      const std::string& label,
      const std::string& description,
      const V& default_value,
      typename std::enable_if<!std::is_arithmetic<U>::value &&
                              !is_specialization_of<std::tuple, U>::value>::type* = nullptr)
      : spec_(InfoTree::make()), is_valid_([](const V&) { return true; }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make(TypeNameRegistry::get_name<U>()));
    spec_->graft("writable", InfoTree::make(is_writable));
    spec_->graft("value", InfoTree::make(static_cast<U>(default_value)));
  }

  template <typename U = T, typename V>
  PropertySpecification(bool is_writable,
                        const std::string& label,
                        const std::string& description,
                        const V& default_value,
                        const V& min_value = std::numeric_limits<U>::min(),
                        const V& max_value = std::numeric_limits<U>::max(),
                        typename std::enable_if<!std::is_same<U, bool>::value &&
                                                std::is_arithmetic<U>::value>::type* = nullptr)
      : spec_(InfoTree::make()), is_valid_([min_value, max_value](const V& val) {
          if (val < min_value || val > max_value) {
            return false;
          }
          return true;
        }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make(TypeNameRegistry::get_name<U>()));
    spec_->graft("writable", InfoTree::make(is_writable));
    spec_->graft("value", InfoTree::make(static_cast<U>(default_value)));
    spec_->graft("min", InfoTree::make(static_cast<U>(min_value)));
    spec_->graft("max", InfoTree::make(static_cast<U>(max_value)));
  }

  template <typename U = bool>
  PropertySpecification(bool is_writable,
                        const std::string& label,
                        const std::string& description,
                        const bool& default_value)
      : spec_(InfoTree::make()), is_valid_([](const bool&) { return true; }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make(TypeNameRegistry::get_name<bool>()));
    spec_->graft("writable", InfoTree::make(is_writable));
    spec_->graft("value", InfoTree::make(default_value));
  }

  template <typename U = char>
  PropertySpecification(bool is_writable,
                        const std::string& label,
                        const std::string& description,
                        const char& default_value)
      : spec_(InfoTree::make()), is_valid_([](const char&) { return true; }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make(TypeNameRegistry::get_name<char>()));
    spec_->graft("writable", InfoTree::make(is_writable));
    spec_->graft("value", InfoTree::make(default_value));
  }

  template <typename U, typename V = Selection<>::index_t>
  PropertySpecification(
      bool is_writable,
      const std::string& label,
      const std::string& description,
      const U& default_value,
      Selection<>::index_t max,
      typename std::enable_if<is_specialization_of<Selection, U>::value>::type* = nullptr)
      : spec_(InfoTree::make()), is_valid_([max](const IndexOrName& ion) {
          if (ion.is_index_ && ion.index_ > max) {
            return false;
          }
          return true;
        }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make("selection"));
    spec_->graft("writable", InfoTree::make(is_writable));
    spec_->graft("value", InfoTree::make(default_value.get()));
    size_t pos = 0;
    for (const auto& it : default_value.get_list()) {
      auto tree = InfoTree::make();
      tree->graft(".label", InfoTree::make(it));
      tree->graft(".id", InfoTree::make(pos));  // overhiding id set by json serializer
      spec_->graft(".values." + std::to_string(pos), tree);
      ++pos;
    }
    spec_->tag_as_array(".values.", true);
  }

  template <typename U = Fraction>
  PropertySpecification(bool is_writable,
                        const std::string& label,
                        const std::string& description,
                        const Fraction& default_value,
                        Fraction::ator_t min_num,
                        Fraction::ator_t min_denom,
                        Fraction::ator_t max_num,
                        Fraction::ator_t max_denom)
      : spec_(InfoTree::make()),
        is_valid_([min_num, min_denom, max_num, max_denom](const Fraction& frac) {
          auto num = frac.numerator();
          if (num < min_num || num > max_num) {
            return false;
          }
          auto denom = frac.denominator();
          if (denom < min_denom || denom > max_denom) {
            return false;
          }
          return true;
        }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make("fraction"));
    spec_->graft("writable", InfoTree::make(is_writable));
    spec_->graft("value", InfoTree::make(default_value));
    spec_->graft("minNumerator", InfoTree::make(min_num));
    spec_->graft("minDenominator", InfoTree::make(min_denom));
    spec_->graft("maxNumerator", InfoTree::make(max_num));
    spec_->graft("maxDenominator", InfoTree::make(max_denom));
  }

  template <typename U = Color>
  PropertySpecification(bool is_writable,
                        const std::string& label,
                        const std::string& description,
                        const Color& default_value)
      : spec_(InfoTree::make()), is_valid_([](const Color&) { return true; }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make("color"));
    spec_->graft("writable", InfoTree::make(is_writable));
    spec_->graft("value", InfoTree::make(default_value));
  }

  template <typename U = T, typename V>
  PropertySpecification(
      bool is_writable,
      const std::string& label,
      const std::string& description,
      const V& default_value,
      typename std::enable_if<is_specialization_of<std::tuple, U>::value>::type* = nullptr)
      : spec_(InfoTree::make()), is_valid_([](const U&) { return true; }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make("tuple"));
    spec_->graft("writable", InfoTree::make(is_writable));
    print_tuple(".value.", default_value);
    spec_->tag_as_array(".value.", true);
  }

  template <typename U = Group,
            typename std::enable_if<std::is_same<U, Group>::value>::type* = nullptr>
  PropertySpecification(const std::string& label, const std::string& description)
      : spec_(InfoTree::make()), is_valid_([](const Group&) { return false; }) {
    spec_->graft("label", InfoTree::make(label));
    spec_->graft("description", InfoTree::make(description));
    spec_->graft("type", InfoTree::make("group"));
    spec_->graft("writable", InfoTree::make(false));
  }

  InfoTree::ptr get_spec() { return spec_; }

  bool is_valid(const TT& val) const { return is_valid_(val); }

  // updating current value
  template <typename U,
            typename std::enable_if<!is_specialization_of<Selection, U>::value &&
                                    !is_specialization_of<std::tuple, U>::value>::type* = nullptr>
  void update_current_value(const U& cur_val) {
    spec_->graft("value", InfoTree::make(cur_val));
  }

  template <typename U,
            typename std::enable_if<is_specialization_of<Selection, U>::value>::type* = nullptr>
  void update_current_value(const U& cur_val) {
    spec_->graft("value", InfoTree::make(cur_val.get()));
  }

  template <typename U,
            typename std::enable_if<is_specialization_of<std::tuple, U>::value>::type* = nullptr>
  void update_current_value(const U& cur_val) {
    print_tuple(".value.", cur_val);
    spec_->tag_as_array(".value.", true);
  }

 private:
  InfoTree::ptr spec_;
  const std::function<bool(const TT&)> is_valid_;

  // writing tuple:
  void print_targs(const std::string&, size_t) {}
  template <typename F, typename... U>
  void print_targs(const std::string& key, size_t pos, F first, U... args) {
    auto tree = InfoTree::make();
    tree->graft(".id", InfoTree::make(pos));  // overhiding id set by json serializer
    tree->graft(".value", InfoTree::make(first));
    tree->graft(".type", InfoTree::make(TypeNameRegistry::get_name<F>()));
    spec_->graft(key + "." + std::to_string(pos), tree);
    print_targs(std::forward<const std::string&>(key), pos + 1, args...);
  }
  template <typename... U, int... S>
  void print_tuple_call(const std::string& key, const std::tuple<U...>& tup, tseq<S...>) {
    print_targs(std::forward<const std::string&>(key), 0, std::get<S>(tup)...);
  }
  template <typename... U>
  void print_tuple(const std::string& key, const std::tuple<U...>& tup) {
    print_tuple_call(std::forward<const std::string&>(key),
                     std::forward<const std::tuple<U...>&>(tup),
                     typename gens<sizeof...(U)>::type());
  }
};

}  // namespace switcher
#endif
