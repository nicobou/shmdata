/* The MIT License (MIT)
 *
 * Copyright (c) 2015 Nicolas Bouillot
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __MAKE_CONSULTABLE_H__
#define __MAKE_CONSULTABLE_H__

#include <iostream>
#include <functional>
#include <type_traits>
#include <cstddef> // size_t
#include <tuple>  // method_traits args and std::get

// selecting method and OPtr for template parameter of consultation method
#define MPtr(_method_ptr)                     \
  decltype(_method_ptr), _method_ptr            

#define OPtr(_PTR, _C, _R, ...)                             \
  decltype(static_cast<_R(_C::*)(__VA_ARGS__)>(_PTR)), _PTR

#define COPtr(_PTR, _C, _R, ...)                               \
  decltype(static_cast<_R(_C::*)(__VA_ARGS__) const>(_PTR)), _PTR

#define COvT(_PTR, _C, _R, ...)                  \
  decltype(static_cast<_R(_C::*)(__VA_ARGS__) const>(_PTR))

#define OvT(_PTR, _C, _R, ...)                \
  decltype(static_cast<_R(_C::*)(__VA_ARGS__)>(_PTR))


namespace nicobou {

// method_traits for method types introspection:
template<typename F, F f>
struct method_traits;
// const
template<typename C,
         typename R,
         typename ...Args,
         R (C::*fn_ptr)(Args... ) const >
struct method_traits<R(C::*)(Args...) const, fn_ptr>{
  using return_type = R;
  using method_t = R(C::*)(Args...) const;
  static constexpr method_t ptr = fn_ptr;
  static constexpr bool is_const = true;
};
// non const
template<typename C,
         typename R,
         typename ...Args,
         R (C::*fn_ptr)(Args... )>
struct method_traits<R(C::*)(Args...), fn_ptr>{
  using return_type = R;
  using method_t = R(C::*)(Args...);
  static constexpr method_t ptr = fn_ptr;
  static constexpr bool is_const = false;
};

// copy a method signature from a class and make the equivalenent
// signature but as a member of an other type
template<typename M, typename T, typename PrefixType>
struct method_convert;
// const
template<typename C,
         typename R,
         typename ...Args,
         typename T,
         typename PrefixType>
struct method_convert<R(C::*)(Args...) const, T, PrefixType>{
  using type = R(T::*)(PrefixType, Args...) const;
};
// non const
template<typename C,
         typename R,
         typename ...Args,
         typename T,
         typename PrefixType>
struct method_convert<R(C::*)(Args...), T, PrefixType>{
  using type = R(T::*)(PrefixType, Args...);
};

// const
template<typename C,
         typename R,
         typename ...Args,
         typename T>
struct method_convert<R(C::*)(Args...) const, T, std::nullptr_t>{
  using type = R(T::*)(Args...) const;
};
// non const
template<typename C,
         typename R,
         typename ...Args,
         typename T>
struct method_convert<R(C::*)(Args...), T, std::nullptr_t>{
  using type = R(T::*)(Args...);
};

}  // namespace nicobou

// encapsulation:
#define Global_wrap(_consult_method,                                    \
                    _encapsulate_return_type,                           \
                    _method_encapsulated)                               \
  static void _consult_method##enable_encaps(){}                        \
                                                                        \
  template<typename EncapsRet = _encapsulate_return_type>               \
  inline EncapsRet                                                      \
  _consult_method##internal_encaps() const {                            \
    return _method_encapsulated();                                      \
  }                                                                     \

#define Selective_hook(_consult_or_fw_method,                           \
                       _delegated_method_type,                          \
                       _delegated_method_ptr,                           \
                       _alternative_method_ptr)                         \
  /*_alternative_method_ptr must not be null*/                          \
                                                                        \
  template<typename DUMMY>                                              \
struct _consult_or_fw_method##alternative_member_<                      \
     _delegated_method_type,                                            \
     _delegated_method_ptr,                                             \
     DUMMY> {                                                           \
  /* cannot static assert here because _alternative_method_ptr */       \
  /* is possibly not yet declared */                                    \
  static typename nicobou::                                             \
    method_convert<_delegated_method_type,                              \
                   _consult_or_fw_method##self_type,                    \
                   _consult_or_fw_method##MapKey_t>::type               \
    get() {                                                             \
      return _alternative_method_ptr;                                   \
    }                                                                   \
};                                                                      \

#define Define_Global_Wrapping(_consult_or_fw_method)                   \
    /* --- global encapsultaion*/                                       \
    /* testing existance of a global ecapsulation method */             \
    template <typename Tested>                                          \
    class _consult_or_fw_method##_has_encaps_method {                   \
      template <typename C> static char test(                           \
          decltype(&C::_consult_or_fw_method##enable_encaps));          \
      template <typename C> static long test(...);                      \
     public:                                                            \
     enum { value = sizeof(test<Tested>(nullptr)) == sizeof(char) };    \
    };                                                                  \
    /*dummy encaspulation method if none has been declared*/            \
    template<typename EncapsRet = bool,                                 \
             typename SelfType = _consult_or_fw_method##self_type,      \
             typename std::                                             \
             enable_if<(!_consult_or_fw_method##_has_encaps_method<     \
                        SelfType>::value)>::type* = nullptr>            \
    inline EncapsRet _consult_or_fw_method##internal_encaps() const {   \
      return true;                                                      \
    }                                                                   \


#define Define_Selective_Hooking(_consult_or_fw_method)                 \
  /* defining default alternative to delegated method invokation */     \
  /* with nullptr method*/                                              \
  template<typename MemberType,                                         \
           MemberType ptr,                                              \
           typename DUMMY = void>                                       \
  struct _consult_or_fw_method##alternative_member_ {                   \
    static typename nicobou::                                           \
    method_convert<MemberType, _consult_or_fw_method##self_type,        \
                   _consult_or_fw_method##MapKey_t>::type               \
    get() {                                                             \
      return nullptr;                                                   \
    };                                                                  \
  };                                                                    \
  /*getting alternative invocation */                                   \
  template<typename MemberType, MemberType ptr>                         \
  static typename nicobou::                                             \
  method_convert<MemberType, _consult_or_fw_method##self_type,          \
                 _consult_or_fw_method##MapKey_t>::type                 \
  _consult_or_fw_method##get_alternative() {                            \
    return                                                              \
    _consult_or_fw_method##alternative_member_<MemberType, ptr>::get(); \
  }                                                                     \


#define Make_access(_self_type,                                         \
                    _member_type,                                       \
                    _member_rawptr,                                     \
                    _consult_method,                                    \
                    _access_flag)                                       \
  static_assert(std::is_class<_member_type>::value,                     \
                "Make_consultable and Make Delegate first "             \
                "argument must be a class");                            \
                                                                        \
  using _consult_method##self_type = _self_type;                        \
                                                                        \
  /*disabling key type for later forward*/                              \
  using _consult_method##MapKey_t = decltype(nullptr);                  \
  /*saving consultable type for the forwarder(s)*/                      \
  using _consult_method##Consult_t = typename                           \
      std::remove_pointer<std::decay<_member_type>::type>::type;        \
                                                                        \
  enum _consult_method##NonConst_t {                                    \
    _consult_method##non_const,                                         \
        _consult_method##const_only                                     \
        };                                                              \
                                                                        \
  Define_Global_Wrapping(_consult_method);                              \
                                                                        \
  Define_Selective_Hooking(_consult_method);                            \
                                                                        \
  /* const and non void return  */                                      \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs>                                             \
  inline typename std::                                                 \
  enable_if<!std::is_same<void,                                         \
                          typename nicobou::                            \
                          method_traits<MMType, fun>::return_type>::value, \
            typename nicobou::method_traits<MMType, fun>::return_type   \
            >::type                                                     \
  _consult_method(BTs ...args) const {                                  \
    static_assert(nicobou::method_traits<MMType, fun>::is_const,        \
                  "consultation is available for const methods only");  \
    auto alt =                                                          \
        _consult_method##get_alternative<decltype(fun), fun>();         \
        if(nullptr != alt)                                              \
          return (this->*alt)(std::forward<BTs>(args)...);              \
        /* __attribute__((unused)) tells compiler encap is not used*/   \
        auto encap __attribute__((unused)) =                            \
            _consult_method##internal_encaps();                         \
            return ((_member_rawptr)->*fun)(std::forward<BTs>(args)...); \
  }                                                                     \
                                                                        \
  /*const and void returning*/                                          \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs>                                             \
  inline typename std::                                                 \
  enable_if<std::is_same<void,                                          \
                         typename nicobou::                             \
                         method_traits<MMType, fun>::return_type>::value\
            >::type                                                     \
  _consult_method(BTs ...args) const {                                  \
    static_assert(nicobou::method_traits<MMType, fun>::is_const,        \
                  "consultation is available for const methods only");  \
    auto alt =                                                          \
        _consult_method##get_alternative<decltype(fun), fun>();         \
        if(nullptr != alt) {                                            \
          (this->*alt)(std::forward<BTs>(args)...);                     \
          return;                                                       \
        }                                                               \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _consult_method##internal_encaps();                             \
        ((_member_rawptr)->*fun)(std::forward<BTs>(args)...);           \
  }                                                                     \
                                                                        \
  /* non const and non void return  */                                  \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           int flag=_consult_method##_access_flag>                      \
  inline typename std::                                                 \
  enable_if<!std::is_same<void,                                         \
                          typename nicobou::                            \
                          method_traits<MMType, fun>::return_type>::value, \
            typename nicobou::method_traits<MMType, fun>::return_type   \
            >::type                                                     \
  _consult_method(BTs ...args) {                                        \
    static_assert(nicobou::method_traits<MMType, fun>::is_const         \
                  || (!nicobou::method_traits<MMType, fun>::is_const    \
                      && flag == _consult_method##NonConst_t::          \
                      _consult_method##non_const),                      \
                  "consultation is available for const methods only"    \
                  " (delegation is disabled)");                         \
        auto alt =                                                      \
            _consult_method##get_alternative<decltype(fun), fun>();     \
        if(nullptr != alt)                                              \
          return (this->*alt)(std::forward<BTs>(args)...);              \
        /* __attribute__((unused)) tells compiler encap is not used*/   \
        auto encap __attribute__((unused)) =                            \
            _consult_method##internal_encaps();                         \
            return ((_member_rawptr)->*fun)(std::forward<BTs>(args)...); \
  }                                                                     \
                                                                        \
  /*non const and void returning*/                                      \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           int flag=_consult_method##_access_flag>                      \
  inline typename std::                                                 \
  enable_if<std::is_same<void,                                          \
                         typename nicobou::                             \
                         method_traits<MMType, fun>::return_type>::value\
            >::type                                                     \
  _consult_method(BTs ...args) {                                        \
    static_assert(nicobou::method_traits<MMType, fun>::is_const         \
                  || (!nicobou::method_traits<MMType, fun>::is_const    \
                      && flag == _consult_method##NonConst_t::          \
                      _consult_method##non_const),                      \
                  "consultation is available for const methods only"    \
                  "(delegation is disabled)");                          \
    auto alt =                                                          \
        _consult_method##get_alternative<decltype(fun), fun>();         \
        if(nullptr != alt){                                             \
          (this->*alt)(std::forward<BTs>(args)...);                     \
          return;                                                        \
        }                                                               \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _consult_method##internal_encaps();                             \
        ((_member_rawptr)->*fun)(std::forward<BTs>(args)...);           \
  }                                                                     \
                                                                        \
  
#define Make_consultable_default(...)           \
  Make_access(__VA_ARGS__, const_only)

#define Make_delegate(...)                      \
  Make_access(__VA_ARGS__, non_const)

// overloading Make_consultable selection Make_access
// and Make_consultable according to number of args
#define Make_consultable_get_overload(_1, _2, _3, _4, _5, NAME,...) NAME
#define Make_consultable(...)                                           \
  Make_consultable_get_overload(                                        \
      __VA_ARGS__,                                                      \
      Make_access,                                                      \
      Make_consultable_default)(__VA_ARGS__)


#define Forward_consultable_full(_self_type,                            \
                                 _member_type,                          \
                                 _member_rawptr,                        \
                                 _consult_method,                       \
                                 _fw_method,                            \
                                 _access_flag)                          \
                                                                        \
  using _fw_method##self_type = _self_type;                             \
                                                                        \
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
  Define_Global_Wrapping(_fw_method);                                   \
                                                                        \
  Define_Selective_Hooking(_fw_method);                                 \
                                                                        \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof...(BTs) for that*/                           \
           typename std::						\
	   enable_if<(sizeof...(BTs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr> \
  inline typename std::                                                 \
  enable_if<!std::is_same<void,                                         \
                          typename nicobou::                            \
                          method_traits<MMType, fun>::return_type>::value, \
            typename nicobou::method_traits<MMType, fun>::return_type   \
            >::type                                                     \
  _fw_method(BTs ...args) const {                                       \
    static_assert(nicobou::method_traits<MMType, fun>::is_const,        \
                  "consultation is available for const methods only");  \
    auto alt = _fw_method##get_alternative<decltype(fun), fun>();       \
    if(nullptr != alt)                                                  \
      return (this->*alt)(std::forward<BTs>(args)...);                  \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _fw_method##internal_encaps();                                  \
    return (_member_rawptr)->                                           \
        _consult_method<MMType, fun>(std::forward<BTs>(args)...);       \
  }                                                                     \
                                                                        \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof...(BTs) for that*/                           \
           typename std::						\
	   enable_if<(sizeof...(BTs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr> \
  inline typename std::                                                 \
  enable_if<std::is_same<void,                                          \
                         typename nicobou::                             \
                         method_traits<MMType, fun>::return_type>::value \
            >::type                                                     \
  _fw_method(BTs ...args) const {                                       \
    static_assert(nicobou::method_traits<MMType, fun>::is_const,        \
                  "consultation is available for const methods only");  \
    auto alt =                                                          \
        _fw_method##get_alternative<decltype(fun), fun>();              \
    if(nullptr != alt) {                                                \
      (this->*alt)(std::forward<BTs>(args)...);                         \
      return;                                                           \
    }                                                                   \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) = _fw_method##internal_encaps(); \
    (_member_rawptr)->                                                  \
        _consult_method<MMType, fun>(std::forward<BTs>(args)...);       \
  }                                                                     \
                                                                        \
                                                                        \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof...(BTs) for that*/                           \
           typename std::						\
	   enable_if<(sizeof...(BTs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr, \
           int flag=_fw_method##_access_flag>                           \
  inline typename std::                                                 \
  enable_if<!std::is_same<void,                                         \
                          typename nicobou::                            \
                          method_traits<MMType, fun>::return_type>::value, \
            typename nicobou::method_traits<MMType, fun>::return_type   \
            >::type                                                     \
  _fw_method(BTs ...args) {                                             \
    static_assert(nicobou::method_traits<MMType, fun>::is_const         \
                  || (!nicobou::method_traits<MMType, fun>::is_const    \
                      && flag == _fw_method##NonConst_t::               \
                      _fw_method##non_const),                           \
                  "Forwarded consultation is available for const "      \
                  "methods only");                                      \
    auto alt =                                                          \
        _fw_method##get_alternative<decltype(fun), fun>();              \
    if(nullptr != alt)                                                  \
      return (this->*alt)(std::forward<BTs>(args)...);                  \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _fw_method##internal_encaps();                                  \
    return (_member_rawptr)->                                           \
        _consult_method<MMType, fun>(std::forward<BTs>(args)...);       \
  }                                                                     \
                                                                        \
                                                                        \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           int flag=_fw_method##_access_flag,                           \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof...(BTs) for that*/                           \
           typename std::						\
	   enable_if<(sizeof...(BTs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr> \
  inline typename std::                                                 \
  enable_if<std::is_same<void,                                          \
                         typename nicobou::                             \
                         method_traits<MMType, fun>::return_type>::value \
            >::type                                                     \
  _fw_method(BTs ...args) {                                             \
    static_assert(nicobou::method_traits<MMType, fun>::is_const         \
                  || (!nicobou::method_traits<MMType, fun>::is_const    \
                      && flag == _fw_method##NonConst_t::               \
                      _fw_method##non_const),                           \
                  "Forwarded consultation is available for const "      \
                  "methods only");                                      \
    auto alt =                                                          \
        _fw_method##get_alternative<decltype(fun), fun>();              \
    if(nullptr != alt){                                                 \
      (this->*alt)(std::forward<BTs>(args)...);                         \
      return;                                                           \
    }                                                                   \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _fw_method##internal_encaps();                                  \
    (_member_rawptr)->                                                  \
          _consult_method<MMType, fun>(std::forward<BTs>(args)...);     \
  }                                                                     \
                                                                        \
                                                                        \
  /*forwarding consult from map if the map key type is defined*/        \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof...(BTs) for that*/                           \
           typename std::						\
	   enable_if<(sizeof...(BTs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      !std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr>	\
  inline typename std::                                                 \
  enable_if<!std::is_same<void,                                         \
                          typename nicobou::                            \
                          method_traits<MMType, fun>::return_type>::value, \
            typename nicobou::method_traits<MMType, fun>::return_type   \
            >::type                                                     \
  _fw_method(_fw_method##MapKey_t key,                                  \
             BTs ...args) const {                                       \
    static_assert(nicobou::method_traits<MMType, fun>::is_const,        \
                  "consultation is available for const methods only");  \
    auto alt =                                                          \
        _fw_method##get_alternative<decltype(fun), fun>();              \
    if(nullptr != alt)                                                  \
      return (this->*alt)(key, std::forward<BTs>(args)...);             \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _fw_method##internal_encaps();                                  \
    return                                                              \
        (_member_rawptr)->                                              \
        _consult_method<MMType, fun>(key, std::forward<BTs>(args)...);  \
  }                                                                     \
                                                                        \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs,                                             \
           /* enable_if work is depends from a template parameter, */   \
           /* using sizeof(BTs) for that*/                              \
           typename std::						\
	   enable_if<(sizeof...(BTs),					\
		      /* if _fw_method##MapKey_t is the same */		\
		      /* type as nullptr then this forward does */	\
		      /* not require map key forwarding*/		\
		      !std::is_same<decltype(nullptr),			\
		      _fw_method##MapKey_t >::value)>::type* = nullptr>	\
  inline typename std::                                                 \
  enable_if<std::is_same<void,                                          \
                         typename nicobou::                             \
                         method_traits<MMType, fun>::return_type>::value \
            >::type                                                     \
  _fw_method(_fw_method##MapKey_t key,                                  \
             BTs ...args) const {                                       \
    static_assert(nicobou::method_traits<MMType, fun>::is_const,        \
                  "consultation is available for const methods only");  \
    auto alt =                                                          \
        _fw_method##get_alternative<decltype(fun), fun>();              \
    if(nullptr != alt) {                                                \
      (this->*alt)(key, std::forward<BTs>(args)...);                     \
      return;                                                           \
    }                                                                   \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _fw_method##internal_encaps();                                  \
    (_member_rawptr)->_consult_method<MMType, fun>(                     \
        key,                                                            \
        std::forward<BTs>(args)...);                                    \
  }                                                                     \


#define Forward_consultable_default(...)                \
  Forward_consultable_full(__VA_ARGS__, const_only)

#define Forward_delegate(...)                           \
  Forward_consultable_full(__VA_ARGS__, non_const)

// overloading Forward_consultable selection Make_access
// and Forawrd_consultable according to number of args
#define Forward_consultable_get_overload(_1, _2, _3, _4, _5, _6, NAME, ...) \
  NAME

#define Forward_consultable(...)                                        \
  Forward_consultable_get_overload(                                     \
      __VA_ARGS__,                                                      \
      Forward_consultable_full,                                         \
      Forward_consultable_default)(__VA_ARGS__)

// returns default constructed R if key not found
// assuming the map is storing shared or unique pointers
#define Forward_consultable_from_associative_container(                 \
    _self_type,                                                         \
    _map_member_type,                                                   \
    _accessor_method,                                                   \
    _map_key_type,                                                      \
    _on_error_construct_ret_method,                                     \
    _consult_method,                                                    \
    _fw_method)                                                         \
    using _fw_method##self_type = _self_type;                           \
                                                                        \
    /*saving key type for later forward*/                               \
    using _fw_method##MapKey_t =                                        \
        const std::decay<_map_key_type>::type &;                        \
                                                                        \
    /*forwarding consultable type for other forwarder(s)*/              \
    using _fw_method##Consult_t = typename                              \
        std::decay<_map_member_type>::type::                            \
        _consult_method##Consult_t;                                     \
                                                                        \
    Define_Global_Wrapping(_fw_method);                                 \
                                                                        \
    Define_Selective_Hooking(_fw_method);                               \
                                                                        \
    template<typename MMType,                                           \
             MMType fun,                                                \
           typename ...BTs>                                             \
    inline typename std::                                               \
    enable_if<!std::is_same<void,                                       \
                            typename nicobou::                          \
                            method_traits<MMType, fun>::return_type>::value, \
              typename nicobou::method_traits<MMType, fun>::return_type \
              >::type                                                   \
    _fw_method(                                                         \
        const _map_key_type &key,                                       \
        BTs ...args) const {                                            \
      static_assert(nicobou::method_traits<MMType, fun>::is_const,      \
                    "consultation is available for const methods only"); \
      auto alt =                                                        \
          _fw_method##get_alternative<decltype(fun), fun>();            \
      if(nullptr != alt)                                                \
        return (this->*alt)(key, std::forward<BTs>(args)...);           \
      /* finding object */                                              \
      auto consultable = _accessor_method (key);                        \
      if (!std::get<0>(consultable))                                    \
        return                                                          \
            _on_error_construct_ret_method<typename nicobou::           \
                                           method_traits<MMType, fun>:: \
                                           return_type>(key);           \
      /*we have the object, continue*/                                  \
      /* __attribute__((unused)) tells compiler encap is not used: */   \
      auto encap __attribute__((unused)) =                              \
          _fw_method##internal_encaps();                                \
      return std::get<1>(consultable)->                                 \
          _consult_method<MMType, fun>(std::forward<BTs>(args)...);     \
  }									\
                                                                        \
  template<typename MMType,                                             \
           MMType fun,                                                  \
           typename ...BTs>                                             \
  inline typename std::                                                 \
  enable_if<std::is_same<void,                                          \
                         typename nicobou::                             \
                         method_traits<MMType, fun>::return_type>::value \
            >::type                                                     \
  _fw_method(                                                           \
      const typename std::decay<_map_key_type>::type &key,              \
      BTs ...args) const {                                              \
    static_assert(nicobou::method_traits<MMType, fun>::is_const,        \
                  "consultation is available for const methods only");  \
    auto alt =                                                          \
        _fw_method##get_alternative<decltype(fun), fun>();              \
    if(nullptr != alt) {                                                \
      (this->*alt)(key, std::forward<BTs>(args)...);                    \
          return;                                                       \
    }                                                                   \
      /* finding object */                                              \
    auto consultable = _accessor_method(key);                           \
    if (!std::get<0>(consultable))                                      \
      return                                                            \
          _on_error_construct_ret_method<typename nicobou::             \
                                         method_traits<MMType, fun>::   \
                                         return_type>(key);             \
    /* __attribute__((unused)) tells compiler encap is not used*/       \
    auto encap __attribute__((unused)) =                                \
        _fw_method##internal_encaps();                                  \
        std::get<1>(consultable)->                                      \
            _consult_method<MMType, fun>(std::forward<BTs>(args)...);   \
  }									\
                                                                        \

#endif
