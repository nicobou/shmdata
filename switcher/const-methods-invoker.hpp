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

#ifndef __SWITCHER_CONST_METHODS_INVOKER_H__
#define __SWITCHER_CONST_METHODS_INVOKER_H__

namespace switcher {

template<typename T>
class ConstMethodsInvoker {
public:
  // exposing T const methods accessible by T instance owner
  template<typename R,       // return type
           typename ...DTs,  // Defined arguments types
           typename ...ATs>  // Arguments types
  R cmi_invoke(R(T::*function)(DTs...) const, ATs ...args)
  {
    std::function<R(T *, DTs...)> fun = function;
    return std::bind(std::move(fun),
                     cmi_get(),
                     std::forward<ATs>(args)...)();
  }

  //
  // template<typename R, typename ...DTs, typename ...ATS>
  // R cmi_invoke(std::function)
  
  // helper for selecting wanted overloaded member
  template<typename R, typename ...ATs>
  using fptr = R(T::*)(ATs...) const;
  
  template<typename R, typename ...ATs, typename ...DTs>
  fptr<R, ATs...> select_overload(R(T::*function)(DTs...) const) {
    return static_cast<fptr<R, ATs...>>(function);
  }

  // disable invokation of non const
  template<typename R, typename ...DTs, typename ...ATs>
  R cmi_invoke(R(T::*function)(DTs...), ATs ...args)
  {
    static_assert(std::is_const<decltype(function)>::value,
                  "ConstMethodsInvoker requires const methods only");
    return (*new R);  // for syntax only since assert should always fail
  }

private:
  // require child to pass instance
  virtual T *cmi_get() = 0;
};

}  // namespace switcher 
#endif
