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

#include <iostream>  // FIXME remove
#include <typeinfo>
#include <algorithm> // find_if
#include <string>
#include <sstream>
#include <functional>

#include "./type-name-registry.hpp"
#include "./information-tree.hpp"
#include "./selection.hpp"

// FIXME a voir avec Francois
// TODO mettre current ? (depuis container peut etre)
// "default value" -> default  // FIXME default is useless ?
// TODO "name" -> "label" 
// nick disparait de enum

namespace switcher {
template<typename T>
class PropertySpecification{
 public:
  PropertySpecification() = delete;

  // typename V is here in order to allow default value to be initiliaed from an other type,
  // e.g. an unsigned int with an int
  
  template <typename U = T, typename V>
  PropertySpecification(const std::string &name,
                        const std::string &description,
                        const V &default_value,
                        typename std::enable_if<
                        !std::is_arithmetic<U>::value 
                        >::type* = nullptr):
      name_(name),
      descr_(description),
      default_value_(default_value),
      spec_(data::Tree::make()){
    spec_->graft("type", data::Tree::make(TypeNameRegistry::get_name<U>()));
    spec_->graft("default", data::Tree::make(static_cast<U>(default_value)));
  }

  template <typename U = T, typename V>
  PropertySpecification(const std::string &name,
                        const std::string &description,
                        const V &default_value,
                        const V &min_value = std::numeric_limits<U>::min(),
                        const V &max_value = std::numeric_limits<U>::max(),
                        typename std::enable_if<
                        !std::is_same<U, bool>::value && 
                        std::is_arithmetic<U>::value 
                        >::type* = nullptr):
      name_(name),
      descr_(description),
      default_value_(default_value),
      spec_(data::Tree::make()){
    spec_->graft("type", data::Tree::make(TypeNameRegistry::get_name<U>()));
    spec_->graft("default", data::Tree::make(static_cast<U>(default_value)));
    spec_->graft("min", data::Tree::make(static_cast<U>(min_value)));
    spec_->graft("max", data::Tree::make(static_cast<U>(max_value)));
  }
  
  template<typename U = bool>
  PropertySpecification(const std::string &name,
                        const std::string &description,
                        const bool &default_value):
      name_(name),
      descr_(description),
      default_value_(default_value),
      spec_(data::Tree::make()){
    spec_->graft("type", data::Tree::make(TypeNameRegistry::get_name<bool>()));
    spec_->graft("default", data::Tree::make(default_value));
  }

  template<typename U = Selection>
  PropertySpecification(const std::string &name,
                        const std::string &description,
                        const Selection &default_value):
      name_(name),
      descr_(description),
      default_value_(default_value),
      spec_(data::Tree::make()){
    spec_->graft("type", data::Tree::make(TypeNameRegistry::get_name<bool>()));
    spec_->graft("default", data::Tree::make(default_value_.get()));
    size_t pos = 0;
    for (const auto &it: default_value.get_list()){
      auto tree = data::Tree::make();
      tree->graft(".label", data::Tree::make(it));
      tree->graft(".value", data::Tree::make(pos));
      spec_->graft(".values." + std::to_string(pos), tree);
      ++pos;
    }
    spec_->tag_as_array(".values.", true);
  }

  data::Tree::ptr get_spec(){
    return spec_;
  }
  
 private:
  const std::string name_;
  const std::string descr_;
  const T default_value_;
  data::Tree::ptr spec_;
};

}  // namespace switcher
#endif
