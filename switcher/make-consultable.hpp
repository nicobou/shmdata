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
                                                                        \
  /*disabling key type for later forward*/                              \
  using _consult_method##MapKeyType = decltype(nullptr);                \
  /*saving consultable type for the forwarder(s)*/                      \
  using _consult_method##ConsultableType = typename                     \
      std::remove_pointer<std::decay<_member_type>::type>::type;        \
                                                                        \
  /* exposing T const methods accessible by T instance owner*/          \
  template<typename R,                                                  \
           typename ...ATs>                                             \
  inline R _consult_method(R(_member_type::*fun)(ATs...) const,         \
			   ATs ...args)	const {                         \
    return (_member_rawptr->*fun)(std::forward<ATs>(args)...);		\
  }                                                                     \
  									\
  template<typename ...ATs>                                             \
  inline void _consult_method(void(_member_type::*fun)(ATs...) const,	\
			      ATs ...args) const {                      \
    (_member_rawptr->*fun)(std::forward<ATs>(args)...);			\
  }                                                                     \
  									\
  /* disable invokation of non const*/                                  \
  template<typename R,                                                  \
           typename ...ATs>                                             \
  R _consult_method(R(_member_type::*function)(ATs...),                 \
                    ATs ...args) const {                                \
    static_assert(std::is_const<decltype(function)>::value,             \
                  "consultation is available for const methods only");  \
    return R();  /* for syntax only since assert should always fail */  \
  }                                                                     \
                                                                        \
  template<typename ...ATs>                                             \
  void _consult_method(void(_member_type::*function)(ATs...),           \
                       ATs ...args) const {                             \
    static_assert(std::is_const<decltype(function)>::value,             \
                  "consultation is available for const methods only");  \
  }



// returns default constructed R if key not found
// assuming the map is storing shared or unique pointers
#define Forward_consultable_from_map(_consult_method,			\
                                     _forward_method,			\
                                     _map_key_type,                     \
                                     _map_member,                       \
                                     _member_type)                      \
                                                                        \
  /*saving key type for later forward*/                                 \
  using _forward_method##MapKeyType =                                   \
      const std::decay<_map_key_type>::type &;                          \
                                                                        \
  /*forwarding consultable type for other forwarder(s)*/                \
  using _forward_method##ConsultableType = typename                     \
      std::decay<_member_type>::type::                                  \
      _consult_method##ConsultableType;                                 \
                                                                        \
  /*shared_ptr & unique_ptr*/                                           \
  /*using _forward_method##ConsultableType =*/				\
  /*  std::decay<_map_value_type>::type::*/                             \
      /*  _consult_method ## ConsultableType; */                        \
  									\
  template<typename R,                                                  \
           typename ...ATs>                                             \
  R _forward_method(							\
      const typename std::decay<_map_key_type>::type &key,              \
      R( _forward_method##ConsultableType ::*function)(ATs...) const,                    \
      ATs ...args) const {                                              \
    auto it = _map_member.find(key);					\
    if (_map_member.end() == it){					\
      static typename std::decay<R>::type r; /*if R is a reference*/	\
      return r;                                                         \
    }									\
    return it->second->_consult_method<R, ATs...>(			\
        std::forward<R( _forward_method##ConsultableType ::*)(ATs...) const>(function),  \
        std::forward<ATs>(args)...);                                    \
  }									\
                                                                        \
  template<typename ...ATs>						\
  void _forward_method(                                                 \
      const typename std::decay<_map_key_type>::type &key,              \
      void( _forward_method##ConsultableType ::*function)(ATs...) const,                 \
      ATs ...args) const {                                              \
    auto it = _map_member.find(key);					\
    if (_map_member.end() == it)					\
      return;								\
    it->second->_consult_method<ATs...>(				\
        std::forward<void( _forward_method##ConsultableType ::*)(ATs...) const>(         \
            function),                                                  \
        std::forward<ATs>(args)...);                                    \
  }									\
                                                                        \
  /* disable invokation of non const*/                                  \
  template<typename R,                                                  \
           typename ...ATs>						\
  R _forward_method(R( _forward_method##ConsultableType ::*function)(ATs...),		\
                    ATs ...args) const {                                \
    static_assert(std::is_const<decltype(function)>::value,		\
                  "consultation is available for const methods only");  \
    return R();  /* for syntax only */                                  \
  }									\
                                                                        \
  template<typename ...ATs>						\
  void _forward_method(void( _forward_method##ConsultableType ::*function)(ATs...),	\
                       ATs ...args) const {                             \
    static_assert(std::is_const<decltype(function)>::value,		\
                  "consultation is available for const methods only");  \
  }


#define Forward_consultable(_member_type,                               \
                            _member_rawptr,                             \
                            _consult_method,                            \
                            _forward_method,                            \
                            _consultable_type)                          \
                                                                        \
  /*forwarding key type*/                                               \
      using _forward_method##MapKeyType =                               \
          typename std::decay<_member_type>::type::                     \
          _consult_method##MapKeyType;                                  \
                                                                        \
                                                                        \
      template<typename R,                                              \
               typename ...ATs>						\
      inline R _forward_method(                                         \
          R(_consultable_type::*function)(ATs...) const,                \
          ATs ...args) const {                                          \
        return _member_rawptr->                                         \
            _consult_method<R, ATs...>(function,                        \
                                   std::forward<ATs>(args)...);         \
  }                                                                     \
                                                                        \
  template<typename ...ATs>						\
  inline void _forward_method(                                          \
      void(_consultable_type::*function)(ATs...) const,                 \
      ATs ...args) const {                                              \
    _member_rawptr->                                                    \
        _consult_method<ATs...>(function,                               \
                                std::forward<ATs>(args)...);            \
  }									\
                                                                        \
                                                                        \
  /*forwarding consult from map if the map key type is defined*/        \
  template<typename R,                                                  \
           typename ...ATs>						\
  inline R _forward_method(                                             \
      /*if _forward_method##MapKeyType is same type as nullptr*/        \
      /* then this forward does not require map key forwarding*/        \
      typename std::enable_if<!std::is_same<decltype(nullptr),          \
      _forward_method##MapKeyType >::value,                             \
      _forward_method##MapKeyType >::type key,                          \
      /*typename std::enable_if<!std::is_class<>::value, T>::type,*/    \
      R(_consultable_type::*function)(ATs...) const,                    \
      ATs ...args) const {                                              \
    return _member_rawptr->                                             \
        _consult_method<R, ATs...>(key,                                 \
                                   function,                            \
                                   std::forward<ATs>(args)...);         \
  }                                                                     \
                                                                        \
  template<typename ...ATs>						\
  inline void _forward_method(                                          \
      typename std::enable_if<!std::is_same<decltype(nullptr),          \
      _forward_method##MapKeyType >::value,                             \
      _forward_method##MapKeyType >::type key,                          \
      void(_consultable_type::*function)(ATs...) const,                 \
      ATs ...args) const {                                              \
    _member_rawptr->                                                    \
        _consult_method<ATs...>(key,                                    \
                                function,                               \
                                std::forward<ATs>(args)...);            \
  }									\
  

#endif
