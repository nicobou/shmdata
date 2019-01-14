/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#ifndef _SHMDATA_TYPE_H_
#define _SHMDATA_TYPE_H_

#include <any>
#include <map>

namespace shmdata {

class Type {
  template <typename T>
  struct empty_t {
    using type = T;
  };

 public:
  Type(const std::string& type);
  std::string name() const;
  std::any get(const std::string& key) const;
  std::map<std::string, std::any> get_properties() const;
  std::string str() const;
  std::string get_parsing_errors() const;
  std::string get_serialization_errors() const;
  template <typename T>
  void set_prop(const std::string& key, const T& value) {
    properties_.emplace(key, value);
  }
  // set a property with a custom type name:
  void set_prop(const std::string& key, const std::string& custom_type, const std::string& value);
  // overload that allows for passing const char* instead of always std::string
  void set_prop(const std::string& key, const char* value);

 private:
  static std::string escape(const std::string& str);
  static std::string unescape(const std::string& str);
  static std::string get_serialized_string_value(const std::any& value);
  std::string name_{};
  std::map<std::string, std::any> properties_{};
  // The str_type member stores type names asssociated to keys.
  // For instance, the pair <"framerate", "fraction"> specify the "framerate" key
  // will be serialized with the type "fraction"
  std::map<std::string, std::string> str_types_{};
  std::string parsing_errors_{};
  mutable std::string serialization_errors_{};
};

}  // namespace shmdata
#endif
