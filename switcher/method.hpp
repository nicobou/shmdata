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

#ifndef __SWITCHER_METHOD_H__
#define __SWITCHER_METHOD_H__

#include <functional>
#include "./bool-any.hpp"
#include "./bool-log.hpp"

namespace switcher {

class MethodBase {
 public:
  using meth_id_t = size_t;
  virtual ~MethodBase() = default;
  static inline meth_id_t id_from_string(const std::string& str) {
    if (!isdigit(*str.begin())) return 0;
    return stoul(str, nullptr, 0);
  }
  virtual BoolLog invoke(const std::string& args) const = 0;  // return a string into BoolLog's msg
  virtual BoolAny invoke_any(const std::string& args) const = 0;  // return an any
};

template <typename M>
class Method : public MethodBase {
 public:
  using lambda_type = M;
  Method(M&& method) : method_(std::forward<M>(method)) {}

  template <typename Tuple, size_t... S>
  decltype(auto) invoke_tuple_impl(Tuple&& t, std::index_sequence<S...>) const {
    return method_(std::get<S>(std::forward<Tuple>(t))...);
  }
  template <typename Tuple>
  decltype(auto) invoke_from_tuple(Tuple&& t) const {
    std::size_t constexpr tSize =
        std::tuple_size<typename std::remove_reference<Tuple>::type>::value;
    return invoke_tuple_impl(std::forward<Tuple>(t), std::make_index_sequence<tSize>());
  }

  template <typename T,
            typename Tup,
            typename std::enable_if<!std::is_same<T, void>::value>::type* = nullptr>
  BoolLog make_valid_boollog(Tup& tup) const {
    return BoolLog(true, serialize::apply<T>(invoke_from_tuple(std::move(tup))));
  }

  template <typename T,
            typename Tup,
            typename std::enable_if<std::is_same<T, void>::value>::type* = nullptr>
  BoolLog make_valid_boollog(Tup&) const {
    return BoolLog(true, std::string());
  }

  BoolLog invoke(const std::string& serialized_tuple) const {
    auto deserialized =
        deserialize::apply<typename method_trait<decltype(method_)>::args_t>(serialized_tuple);
    if (!deserialized.first)
      return BoolLog(
          false,
          std::string("invoke failed to deserialize following arguments: ") + serialized_tuple);
    return make_valid_boollog<typename method_trait<M>::return_t>(deserialized.second);
  }

  template <typename T,
            typename Tup,
            typename std::enable_if<!std::is_same<T, void>::value>::type* = nullptr>
  BoolAny make_valid_boolany(Tup& tup) const {
    return BoolAny(Any(invoke_from_tuple(std::move(tup))));
  }

  template <typename T,
            typename Tup,
            typename std::enable_if<std::is_same<T, void>::value>::type* = nullptr>
  BoolAny make_valid_boolany(Tup&) const {
    return BoolAny(Any());
  }

  BoolAny invoke_any(const std::string& serialized_tuple) const {
    auto deserialized =
        deserialize::apply<typename method_trait<decltype(method_)>::args_t>(serialized_tuple);
    if (!deserialized.first)
      return BoolAny(
          Any(),
          false,
          std::string("invoke failed to deserialize following arguments: ") + serialized_tuple);
    return make_valid_boolany<typename method_trait<M>::return_t>(deserialized.second);
  }

 private:
  M method_;
};

}  // namespace switcher
#endif
