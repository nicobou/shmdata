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
 public:
  Type(const std::string& type);
  std::string name() const;
  std::any get(const std::string& key) const;
  std::map<std::string,std::any> get_properties() const;
  std::string str() const;
  std::string get_parsing_errors() const;
  std::string get_serialization_errors() const;
  template <typename T> void set_prop(const std::string& key,const T& value) {
    properties_.emplace(key, value);
  }
 private:
  static std::string escape(const std::string& str);
  static std::string unescape(const std::string& str);
  std::string name_{};
  std::map<std::string,std::any> properties_{};
  std::string parsing_errors_{};
  mutable std::string serialization_errors_{};
};

}  // namespace shmdata
#endif
