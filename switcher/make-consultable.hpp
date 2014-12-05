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

#ifndef __SWITCHER_MAKE_CONSULTABLE_H__
#define __SWITCHER_MAKE_CONSULTABLE_H__

#include <functional>

#define Make_consultable(_member_type, _member_rawptr, _consult_method) \
  static_assert(std::is_class<_member_type>::value,                     \
                "Make_consultable 2nd arg must be a class");            \
  /*FIXME check _member_ptr is a raw ptr */                             \
                                                                        \
  /*defining member type for being retrieved by forwarders*/            \
  using _consult_method##ConsultableType = _member_type;                \
                                                                        \
  /* exposing T const methods accessible by T instance owner*/          \
  template<typename R,                                                  \
           typename ...ATs>                                             \
  R _consult_method(R(_member_type::*function)(ATs...) const,           \
                    ATs ...args)                                        \
  {                                                                     \
    std::function<R(_member_type *, ATs...)> fun = function;            \
    return std::bind(std::move(fun),                                    \
                     _member_rawptr,                                    \
                     std::forward<ATs>(args)...)();                     \
  }                                                                     \
                                                                        \
  template<typename ...ATs>                                             \
  void                                                                  \
  _consult_method(void(_member_type::*function)(ATs...) const,          \
                  ATs ...args)                                          \
  {                                                                     \
    std::function<void(_member_type *, ATs...)> fun =                   \
        function;                                                       \
    std::bind(std::move(fun),                                           \
              _member_rawptr,                                           \
              std::forward<ATs>(args)...)();                            \
  }                                                                     \
                                                                        \
  /* disable invokation of non const*/                                  \
  template<typename R,                                                  \
           typename ...ATs>                                             \
  R _consult_method(R(_member_type::*function)(ATs...),                 \
                    ATs ...args)                                        \
  {                                                                     \
    static_assert(std::is_const<decltype(function)>::value,             \
                  "consultation is available for const methods only");  \
    return R();  /* for syntax only since assert should always fail */  \
  }                                                                     \
                                                                        \
  template<typename ...ATs>                                             \
  void _consult_method(void(_member_type::*function)(ATs...),           \
                       ATs ...args)                                     \
  {                                                                     \
    static_assert(std::is_const<decltype(function)>::value,             \
                  "consultation is available for const methods only");  \
  }


// class that having objects in a map container member,
// returning default constructed R if key not found
// assuming the map is storing shared or unique pointers
#define Forward_consultable_from_map(_consult_method_name,              \
                                     _forward_method_name,              \
                                     _map_key_type,                     \
                                     _map_member,                       \
                                     _consultable_type)                 \
                                                                        \
  /*shared_ptr & unique_ptr*/                                           \
      /*using _forward_method_name##ConsultableType =*/                 \
  /*  std::decay<_map_value_type>::type::*/                             \
  /*  _consult_method_name ## ConsultableType; */                       \
                                                                        \
  template<typename R,                                                  \
           typename ...ATs>                                             \
  R _forward_method_name(                                               \
      const typename std::decay<_map_key_type>::type &key,              \
      R(_consultable_type::*function)(ATs...) const,                    \
      ATs ...args) {                                                    \
    auto it = _map_member.find(key);                                    \
    if (_map_member.end() == it){                                       \
      static typename std::decay<R>::type r; /*if R is a reference*/    \
      return r;                                                         \
    }                                                                   \
    return it->second->_consult_method_name<R, ATs...>(                 \
        std::forward<R(_consultable_type::*)(ATs...) const>(function),  \
        std::forward<ATs>(args)...);                                    \
  }                                                                     \
                                                                        \
  template<typename ...ATs>                                             \
  void _forward_method_name(                                            \
      const typename std::decay<_map_key_type>::type &key,              \
      void(_consultable_type::*function)(ATs...) const,                 \
      ATs ...args) {                                                    \
    auto it = _map_member.find(key);                                    \
    if (_map_member.end() == it)                                        \
      return;                                                           \
    it->second->_consult_method_name<ATs...>(                           \
        std::forward<void(_consultable_type::*)(ATs...) const>(         \
            function),                                                  \
        std::forward<ATs>(args)...);                                    \
  }                                                                     \
                                                                        \
  /* disable invokation of non const*/                                  \
  template<typename R,                                                  \
           typename ...ATs>                                             \
  R _forward_method_name(R(_consultable_type::*function)(ATs...),       \
                         ATs ...args)                                   \
  {                                                                     \
    static_assert(std::is_const<decltype(function)>::value,             \
                  "consultation is available for const methods only");  \
    return R();  /* for syntax only */                                  \
  }                                                                     \
                                                                        \
  template<typename ...ATs>                                             \
  void _forward_method_name(void(_consultable_type::*function)(ATs...), \
                            ATs ...args)                                \
  {                                                                     \
    static_assert(std::is_const<decltype(function)>::value,             \
                  "consultation is available for const methods only");  \
  }


#endif
