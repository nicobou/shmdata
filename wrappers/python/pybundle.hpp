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
 *
 */

#ifndef __SWITCHER_PYBUNDLE_H__
#define __SWITCHER_PYBUNDLE_H__

#include <Python.h>

class BundleManager {
 public:
  using BundleManagerObject = struct {
    PyObject_HEAD PyObject* pyswitch{nullptr};
    PyObject* registry{nullptr};
  };

  static PyTypeObject pyType;
  static PySequenceMethods tp_as_sequence;
  static PyMethodDef tp_methods[];

 private:
  static PyObject* tp_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
  static int tp_init(BundleManagerObject* self, PyObject* args, PyObject* kwds);
  static void tp_dealloc(BundleManagerObject* self);

  // __get__
  static PyObject* tp_descr_get(BundleManagerObject* self, PyObject* args, PyObject* kwds);

  // __len__
  static PyObject* sq_length(BundleManagerObject* self);
  // __getitem__
  static PyObject* sq_item(BundleManagerObject* self, Py_ssize_t i);
  // __setitem__
  static PyObject* sq_ass_item(BundleManagerObject* self, PyObject* key, PyObject* v);

  // __repr__
  static PyObject* tp_repr(BundleManagerObject* self, PyObject* args, PyObject* kwds);

  // append
  static PyObject* append(BundleManagerObject* self, PyObject* args, PyObject* kwds);
  // create
  static PyObject* create(BundleManagerObject* self, PyObject* args, PyObject* kwds);
  // pop
  static PyObject* pop(BundleManagerObject* self, PyObject* args, PyObject* kwds);
};

#endif
