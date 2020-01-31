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

#include <map>
#include <string>
#include <typeinfo>
#include <vector>

namespace switcher {

class TypeNameRegistry {
 public:
  TypeNameRegistry() = delete;
  template <typename T>
  static std::string get_name() {
    return type_name_registry_[typeid(T).hash_code()];
  }

  template <typename T>
  static void unfold_tuple(std::vector<std::string>& vec) {
    vec.emplace_back(get_name<T>());
  }

  template <typename T, typename T2, typename... U>
  static void unfold_tuple(std::vector<std::string>& vec) {
    vec.emplace_back(get_name<T>());
    unfold_tuple<T2, U...>(vec);
  }

  template <typename Tup, size_t... S>
  static void prepare_unfold_tuple(std::vector<std::string>& vec, std::index_sequence<S...>) {
    unfold_tuple<typename std::tuple_element<S, Tup>::type...>(vec);
  }

  template <typename Tup>
  static std::vector<std::string> get_names() {
    std::vector<std::string> res;
    std::size_t constexpr tuple_size =
        std::tuple_size<typename std::remove_reference<Tup>::type>::value;
    prepare_unfold_tuple<Tup>(res, std::make_index_sequence<tuple_size>());
    return res;
  }

 private:
  using registry_t = std::map<size_t, std::string>;
  static registry_t type_name_registry_;
};

template <>
std::vector<std::string> TypeNameRegistry::get_names<std::tuple<>>();

}  // namespace switcher
#endif
