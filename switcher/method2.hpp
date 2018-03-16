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

#ifndef __SWITCHER_METHOD2_H__
#define __SWITCHER_METHOD2_H__

#include <functional>

namespace switcher {

class MethodBase {
 public:
  using meth_id_t = size_t;
  MethodBase() = delete;
  MethodBase(size_t type_hash) : type_hash_(type_hash) {}
  virtual ~MethodBase() = default;
  static inline meth_id_t id_from_string(const std::string& str) {
    if (!isdigit(*str.begin())) return 0;
    return stoul(str, nullptr, 0);
  }

 private:
  size_t type_hash_;
};

template <typename M>
class Method2 : public MethodBase {
 public:
  using type = M;
  Method2(M&& method) : MethodBase(typeid(M).hash_code()), method_(std::forward<M>(method)) {}

  template <typename Tuple, size_t... S>
  decltype(auto) invoke_tuple_impl(Tuple&& t, std::index_sequence<S...>) {
    return method_(std::get<S>(std::forward<Tuple>(t))...);
  }
  template <typename Tuple>
  decltype(auto) invoke_from_tuple(Tuple&& t) {
    std::size_t constexpr tSize =
        std::tuple_size<typename std::remove_reference<Tuple>::type>::value;
    return invoke_tuple_impl(std::forward<Tuple>(t), std::make_index_sequence<tSize>());
  }

 private:
  M method_;
};

}  // namespace switcher
#endif
