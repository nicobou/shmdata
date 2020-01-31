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

#ifndef __SWITCHER_METHOD_TRAIT_H__
#define __SWITCHER_METHOD_TRAIT_H__

#include <tuple>

namespace switcher {
namespace method_detail {
template <class Ret, class Cls, class IsMutable, class... Args>
struct types {
  using is_mutable = IsMutable;
  enum { arity = sizeof...(Args) };
  using return_t = Ret;
  using args_t = std::tuple<typename std::decay<Args>::type...>;
  template <size_t i>
  struct arg {
    typedef typename std::tuple_element<i, std::tuple<Args...>>::type type;
  };
};
}

template <class Ld>
struct method_trait : method_trait<decltype(&Ld::operator())> {};

template <class Ret, class Cls, class... Args>
struct method_trait<Ret (Cls::*)(Args...)>
    : method_detail::types<Ret, Cls, std::true_type, Args...> {};

template <class Ret, class Cls, class... Args>
struct method_trait<Ret (Cls::*)(Args...) const>
    : method_detail::types<Ret, Cls, std::false_type, Args...> {};

}  // namespace switcher
#endif
