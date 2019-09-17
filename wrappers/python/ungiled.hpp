/*
 * This file is part of switcher python wrapper.
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

#ifndef __SWITCHER_UNGILED_H__
#define __SWITCHER_UNGILED_H__

#include <Python.h>  // according to python doc, this *must* be the first include
#include <functional>

namespace pyquid {

// let python3 do some stuff while running non python cpp code :
template <typename T>
T ungiled(const std::function<T()>& fun) {
  T res;
  {
    auto* state = PyEval_SaveThread();
    res = fun();
    PyEval_RestoreThread(state);
  }
  return res;
}

}  // namespace pyquid
#endif
