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

#ifndef __SWITCHER_TYPE_NAME_REGISTRY_H__
#define __SWITCHER_TYPE_NAME_REGISTRY_H__

#include <string>
#include <map>
#include <typeinfo>

namespace switcher{

class TypeNameRegistry{
 public:
  TypeNameRegistry() = delete;
  template<typename T>
  static std::string get_name(){
    return type_name_registry_[typeid(T).hash_code()];
  }
 private:
  using registry_t = std::map<size_t, std::string>;
  static registry_t type_name_registry_;
};

}  // namespace switcher
#endif
