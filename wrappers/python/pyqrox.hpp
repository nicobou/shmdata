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

#ifndef __SWITCHER_PYQROX_H__
#define __SWITCHER_PYQROX_H__

#include <Python.h>  // according to python doc, this *must* be the first include
#include "switcher/quiddity/quiddity-qrox.hpp"

using namespace switcher;

class pyQrox {
 public:
  using pyQroxObject = struct { PyObject_HEAD std::unique_ptr<quid::Qrox> qrox{}; };

  static PyTypeObject pyType;
  static PyMethodDef pyQrox_methods[];

 private:
  // Boilerplate
  static PyObject* Qrox_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/);
  static int Qrox_init(pyQroxObject* self, PyObject* /*args*/, PyObject* /*kwds*/);
  static void Qrox_dealloc(pyQroxObject* self);
  static PyObject* id(pyQroxObject* self);
  static PyObject* name(pyQroxObject* self);
  static PyObject* quid(pyQroxObject* self);
};
#endif
