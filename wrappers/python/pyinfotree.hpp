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

#ifndef __SWITCHER_PYINFOTREE_H__
#define __SWITCHER_PYINFOTREE_H__

#include <Python.h>  // according to python doc, this *must* be the first include
#include "switcher/quiddity/quiddity-qrox.hpp"

using namespace switcher;

class pyInfoTree {
 public:
  using pyInfoTreeObject = struct {
    PyObject_HEAD InfoTree* tree{};
    InfoTree::ptr keepAlive{};
  };

  static PyTypeObject pyType;
  static PyMethodDef pyInfoTree_methods[];
  static PyObject* any_to_pyobject(const Any& any);

  static PyObject* make_pyobject_from_c_ptr(InfoTree* tree, bool do_copy);

 private:
  // Boilerplate
  static PyObject* InfoTree_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/);
  static int InfoTree_init(pyInfoTreeObject* self, PyObject* /*args*/, PyObject* /*kwds*/);
  static void InfoTree_dealloc(pyInfoTreeObject* self);
  static PyObject* empty(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* prune(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* copy(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* graft(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* json(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* tag_as_array(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_child_keys(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_key_values(pyInfoTreeObject* self, PyObject* args, PyObject* kwds);
};
#endif
