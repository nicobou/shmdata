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

#ifndef __MAKE_CONSULTABLE_H__
#define __MAKE_CONSULTABLE_H__

#include <functional>
#include <type_traits>


#define Make_consultable_full(_member_type,                             \
                              _member_rawptr,                           \
                              _consult_method,                          \
                              _access_flag)                             \
  static_assert(std::is_class<_member_type>::value,                     \
                "Make_consultable 1st arg must be a class");            \
                                                                        \
  /*disabling key type for later forward*/                              \
  using _consult_method##MapKey_t = decltype(nullptr);                  \
  /*saving consultable type for the forwarder(s)*/                      \
  using _consult_method##Consult_t = typename                           \
      std::remove_pointer<std::decay<_member_type>::type>::type;        \
                                                                        \
  enum  _consult_method##NonConst_t {                                   \
    _consult_method##non_const,                                         \
        _consult_method##const_only                                     \
        };                                                              \
                                                                        \
  /* exposing T const methods accessible by T instance owner*/          \
  template<typename R,                                                  \
           typename ...ATs,                                             \
           typename ...BTs>                                             \
  inline R _consult_method(R(_member_type::*fun)(ATs...) const,         \
			   BTs ...args)	const {                         \
    return ((_member_rawptr)->*fun)(std::forward<BTs>(args)...);        \
  }                                                                     \
                                                                        \
  template<typename ...ATs,                                             \
           typename ...BTs>                                             \
  inline void _consult_method(void(_member_type::*fun)(ATs...) const,	\
			      BTs ...args) const {                      \
    ((_member_rawptr)->*fun)(std::forward<BTs>(args)...);               \
  }                                                                     \
                                                                        \
  /* disable invokation of non const*/                                  \
  template<typename R,                                                  \
           typename ...ATs,                                             \
           typename ...BTs,                                             \
           int i=_consult_method##_access_flag>                         \
  inline R _consult_method(R(_member_type::*fun)(ATs...),               \
                           BTs ...args) {                               \
    static_assert( i == _consult_method##NonConst_t::                   \
                   _consult_method##non_const,                          \
                   "consultation is available for const methods only "  \
                   "and delegation is disabled");                       \
    return ((_member_rawptr)->*fun)(std::forward<BTs>(args)...);        \
  }                                                                     \
                                                                        \
  template<typename ...ATs,                                             \
           typename ...BTs,                                             \
           int i=_consult_method##_access_flag>                         \
  void _consult_method(void(_member_type::*fun)(ATs...),                \
                       BTs ...args) {                                   \
    static_assert(i == _consult_method##NonConst_t::                    \
                  _consult_method##non_const,                           \
                   "consultation is available for const methods only"   \
                  "and delegation is disabled");                        \
    ((_member_rawptr)->*fun)(std::forward<BTs>(args)...);               \
  }                                                                     \
  
#define Make_consultable_default(...)                   \
  Make_consultable_full(__VA_ARGS__, const_only)

#define Make_delegate(...)                      \
  Make_consultable_full(__VA_ARGS__, non_const)

// overloading Make_consultable selection Make_consultable_full
// and Make_consultable according to number of args
#define Make_consultable_get_overload(_1, _2, _3, _4, NAME,...) NAME
#define Make_consultable(...)                                           \
  Make_consultable_get_overload(                                        \
      __VA_ARGS__,                                                      \
      Make_consultable_full,                                            \
      Make_consultable_default)(__VA_ARGS__)


#define Forward_consultable_full(_member_type,                          \
                                 _member_rawptr,                        \
                                 _consult_method,                       \
                                 _fw_method,                            \
                                 _access_flag)                          \
  /*forwarding key type*/                                               \
  using _fw_method##MapKey_t =                                          \
      typename std::decay<_member_type>::type::                         \
      _consult_method##MapKey_t;                                        \
                                                                        \
  /*forwarding consultable type for other forwarder(s)*/                \
  using _fw_method##Consult_t = typename                                \
      std::decay<_member_type>::type::                                  \
      _consult_method##Consult_t;                                       \
                                                                        \
  enum  _fw_method##NonConst_t {                                        \
    _fw_method##non_const,                                              \
        _fw_method##const_only                                          \
        };                                                              \
                                                                        \
  template<typename R,                                                  \
           typename ...ATs,                                             \
           typename ...BTs>                                             \
  inline R _fw_method(                                                  \
      R( _fw_method##Consult_t ::*function)(ATs...) const,              \
      BTs ...args) const {                                              \
    return (_member_rawptr)->                                           \
        _consult_method<R, ATs...>(                                     \
            std::forward<R( _fw_method##Consult_t ::*)(ATs...) const>(  \
                function),                                              \
            std::forward<BTs>(args)...);                                \
  }                                                                     \
                                                                        \
  template<typename ...ATs,                                             \
           typename ...BTs>                                             \
  inline void _fw_method(                                               \
      void( _fw_method##Consult_t ::*function)(ATs...) const,           \
      BTs ...args) const {                                              \
    (_member_rawptr)->_consult_method<ATs...>(                          \
        std::forward<void( _fw_method##Consult_t ::*)(ATs...) const>(   \
            function),                                                  \
        std::forward<BTs>(args)...);                                    \
  }									\
                                                                        \
  template<typename R,                                                  \
           typename ...ATs,                                             \
           typename ...BTs,                                             \
           int i=_fw_method##_access_flag>                              \
  inline R _fw_method(                                                  \
      R( _fw_method##Consult_t ::*function)(ATs...),                    \
      BTs ...args) {                                                    \
    static_assert( i == _fw_method##NonConst_t::                        \
                   _fw_method##non_const,                               \
                   "Forwarded consultation is available for const "     \
                   "methods only and non_const has not been set");      \
    return (_member_rawptr)->                                           \
        _consult_method<R, ATs...>(                                     \
            std::forward<R( _fw_method##Consult_t ::*)(ATs...)>(        \
                function),                                              \
            std::forward<BTs>(args)...);                                \
  }                                                                     \
                                                                        \
  template<typename ...ATs,                                             \
           typename ...BTs,                                             \
           int i=_fw_method##_access_flag>                              \
  inline void _fw_method(                                               \
      void( _fw_method##Consult_t ::*function)(ATs...),                 \
      BTs ...args) {                                                    \
    static_assert( i == _fw_method##NonConst_t::                        \
                   _fw_method##non_const,                               \
                   "Forwarded consultation is available for const "     \
                   "methods only and non_const has not been set");      \
    (_member_rawptr)->_consult_method<ATs...>(                          \
        std::forward<void( _fw_method##Consult_t ::*)(ATs...)>(         \
            function),                                                  \
        std::forward<BTs>(args)...);                                    \
  }									\
                                                                        \
                                                                        \
  /*forwarding consult from map if the map key type is defined*/        \
  template<typename R,                                                  \
           typename ...ATs,                                             \
           typename ...BTs,                                             \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof...(ATs) for that*/                           \
           typename std::						\
	   enable_if<(sizeof...(ATs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      !std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr>	\
  inline R _fw_method(                                                  \
      _fw_method##MapKey_t key,                                         \
      /*typename std::enable_if<!std::is_class<>::value, T>::type,*/    \
      R( _fw_method##Consult_t ::*function)(ATs...) const,              \
      BTs ...args) const {                                              \
    return (_member_rawptr)->                                           \
        _consult_method<R, ATs...>(                                     \
            std::forward< _fw_method##MapKey_t >(key),                  \
            std::forward<R( _fw_method##Consult_t ::*)(ATs...) const>(  \
                function),                                              \
            std::forward<BTs>(args)...);                                \
  }                                                                     \
                                                                        \
  template<typename ...ATs,                                             \
           typename ...BTs,                                             \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof(ATs) for that*/                              \
           typename std::						\
	   enable_if<(sizeof...(ATs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      !std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr>	\
  inline void _fw_method(                                               \
      _fw_method##MapKey_t key,                                         \
      void( _fw_method##Consult_t ::*function)(ATs...) const,           \
      BTs ...args) const {                                              \
    (_member_rawptr)->_consult_method<ATs...>(                          \
        std::forward< _fw_method##MapKey_t >(key),                      \
        std::forward<void( _fw_method##Consult_t ::*)(ATs...) const>(   \
            function),                                                  \
        std::forward<BTs>(args)...);                                    \
  }									\

#define Forward_consultable_default(...)                   \
  Forward_consultable_full(__VA_ARGS__, const_only)

#define Forward_delegate(...)                           \
  Forward_consultable_full(__VA_ARGS__, non_const)

// overloading Forward_consultable selection Make_consultable_full
// and Forawrd_consultable according to number of args
#define Forward_consultable_get_overload(_1, _2, _3, _4, _5, NAME,...)  \
  NAME
#define Forward_consultable(...)                                        \
  Forward_consultable_get_overload(                                     \
      __VA_ARGS__,                                                      \
      Forward_consultable_full,                                         \
      Forward_consultable_default)(__VA_ARGS__)

// returns default constructed R if key not found
// assuming the map is storing shared or unique pointers
#define Forward_consultable_from_map(_map_key_type,                     \
                                     _map_member_type,                  \
                                     _map_member,                       \
                                     _consult_method,			\
                                     _fw_method)			\
                                                                        \
  /*saving key type for later forward*/                                 \
  using _fw_method##MapKey_t =                                          \
      const std::decay<_map_key_type>::type &;                          \
                                                                        \
  /*forwarding consultable type for other forwarder(s)*/                \
  using _fw_method##Consult_t = typename                                \
      std::decay<_map_member_type>::type::                              \
      _consult_method##Consult_t;                                       \
                                                                        \
  template<typename R,                                                  \
           typename ...ATs,                                             \
           typename ...BTs>                                             \
  R _fw_method(                                                         \
      const typename std::decay<_map_key_type>::type &key,              \
      R( _fw_method##Consult_t ::*function)(ATs...) const,              \
      BTs ...args) const {                                              \
    auto it = _map_member.find(key);					\
    if (_map_member.end() == it){					\
      static typename std::decay<R>::type r; /*if R is a reference*/	\
      return r;                                                         \
    }									\
    return it->second->_consult_method<R, ATs...>(			\
        std::forward<R( _fw_method##Consult_t ::*)(ATs...) const>(      \
            function),                                                  \
        std::forward<BTs>(args)...);                                    \
  }									\
                                                                        \
  template<typename ...ATs,                                             \
           typename ...BTs>                                             \
  void _fw_method(                                                      \
      const typename std::decay<_map_key_type>::type &key,              \
      void( _fw_method##Consult_t ::*function)(ATs...) const,           \
      BTs ...args) const {                                              \
    auto it = _map_member.find(key);					\
    if (_map_member.end() == it)					\
      return;								\
    it->second->_consult_method<ATs...>(				\
        std::forward<void( _fw_method##Consult_t ::*)(ATs...) const>(   \
            function),                                                  \
        std::forward<BTs>(args)...);                                    \
  }									\
                                                                        \
  /* disable invokation of non const*/                                  \
  template<typename R,                                                  \
           typename ...ATs,                                             \
           typename ...BTs>                                             \
  R _fw_method(R( _fw_method##Consult_t ::*function)(ATs...),           \
               BTs ...) const {						\
    static_assert(std::is_const<decltype(function)>::value,		\
                  "consultation is available for const methods only");  \
    return R();  /* for syntax only */                                  \
  }									\
                                                                        \
  template<typename ...ATs,                                             \
           typename ...BTs>						\
  void _fw_method(void( _fw_method##Consult_t ::*function)(ATs...),     \
                  BTs ...) const {					\
    static_assert(std::is_const<decltype(function)>::value,		\
                  "consultation is available for const methods only");  \
  }

#endif
