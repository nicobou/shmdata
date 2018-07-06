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

#ifndef __SWITCHER_PYQUIDDITY_H__
#define __SWITCHER_PYQUIDDITY_H__

#include <Python.h>  // according to python doc, this *must* be the first include
#include <map>
#include "switcher/quiddity.hpp"

using namespace switcher;

class pyQuiddity {
 public:
  using registered_signals_t = std::map<SContainer::sig_id_t, SContainer::register_id_t>;
  using registered_callbacks_t = std::map<SContainer::sig_id_t, PyObject*>;
  using pyQuiddityObject = struct {
    PyObject_HEAD Quiddity* quid{nullptr};
    std::unique_ptr<registered_signals_t> registered_signals{};
    std::unique_ptr<registered_callbacks_t> registered_callbacks{};
  };

  static PyTypeObject pyType;
  static PyMethodDef pyQuiddity_methods[];

 private:
  // Boilerplate
  static PyObject* Quiddity_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/);
  static int Quiddity_init(pyQuiddityObject* self, PyObject* /*args*/, PyObject* /*kwds*/);
  static void Quiddity_dealloc(pyQuiddityObject* self);
  static PyObject* set(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* invoke(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* make_shmpath(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  // access to user tree
  static PyObject* get_user_tree(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  // access to quiddity InfoTree
  static PyObject* get_info(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_info_tree_as_json(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  // signals
  static PyObject* subscribe(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* unsubscribe(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_signal_id(pyQuiddityObject* self, PyObject* args, PyObject* kwds);
};
#endif
