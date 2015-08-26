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
    spec_->graft("default value", data::Tree::make(static_cast<U>(default_value)));
    
    std::cout << "type: " << TypeNameRegistry::get_name<U>()
              << " not numeric, default value: "
              << default_value
              << std::endl;
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

    std::cout << "type: " << TypeNameRegistry::get_name<U>()
              << " default value: " << std::to_string(default_value)
              << " min: " << std::to_string(min_value)
              << " max: " << std::to_string(max_value)
              << std::endl;
  }
  
  template<typename U = bool>
  PropertySpecification(const std::string &name,
                        const std::string &description,
                        const bool &default_value):
      name_(name),
      descr_(description),
      default_value_(default_value),
      spec_(data::Tree::make()){
    spec_->graft("type", data::Tree::make(TypeNameRegistry::get_name<U>()));
    spec_->graft("default value", data::Tree::make(static_cast<U>(default_value)));
    
    std::cout  << std::boolalpha
               << "type: " << TypeNameRegistry::get_name<T>()
               << " default value: " << default_value
               << std::endl;
  }

  data::Tree::ptr get_spec(){
    return spec_;
  }
  
 private:
  const std::string name_;
  const std::string descr_;
  const T default_value_;
  data::Tree::ptr spec_; // need a tree for enum like types
};

}  // namespace switcher
#endif
