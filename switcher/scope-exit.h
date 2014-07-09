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

#ifndef __SWITCHER_SCOPE_EXIT_H__
#define __SWITCHER_SCOPE_EXIT_H__

//scope exit implementation from Alexandrescu's talk at NDC 2014

#include <utility> 

namespace scope_guard
{

  template <typename Fun>
    class ScopeGuard
    {
    public:
      ScopeGuard (Fun &&fun) :
        fun_ (std::move (fun))
      {}
	~ScopeGuard () {fun_ ();}
    private:
	Fun fun_;
    };

  enum class ScopeGuardOnExit {};
  template <typename Fun>
    ScopeGuard<Fun>
    operator+ (ScopeGuardOnExit, Fun &&fn) {
    return ScopeGuard<Fun> (std::forward<Fun> (fn));
  }

}// end namespace scope_guard

#define CONCATENATE_IMPL(s1, s2) s1##s2
#define CONCATENATE(s1, s2) CONCATENATE_IMPL(s1, s2)

//could replace __LINE__ with __COUNTER__ but not always available
#define On_scope_exit \
  auto CONCATENATE(on_scope_exit_var, __LINE__) \
    = ::scope_guard::ScopeGuardOnExit () + [&]()

#endif  //__SWITCHER_SCOPE_EXIT_H__
