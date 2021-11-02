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

#ifndef __SWITCHER_PYWCLAW_H__
#define __SWITCHER_PYWCLAW_H__

#include <Python.h>  // according to python doc, this *must* be the first include

#include "switcher/quiddity/claw/claw.hpp"
#include "switcher/quiddity/quiddity.hpp"

using namespace switcher;
using namespace quiddity;
using namespace claw;

class pyWriterClaw {
 public:
  using pyWriterClawObject = struct {
    PyObject_HEAD std::weak_ptr<Quiddity> quid{};
    swid_t id;
    long invalid_id;
  };

  static PyTypeObject pyType;
  static PyMethodDef pyWriterClaw_methods[];

 private:
  // Boilerplate
  static PyObject* WriterClaw_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/);
  static int WriterClaw_init(pyWriterClawObject* self, PyObject* /*args*/, PyObject* /*kwds*/);
  static void WriterClaw_dealloc(pyWriterClawObject* self);
  static PyObject* label(pyWriterClawObject* self, PyObject* args, PyObject* kwds);
  static PyObject* id(pyWriterClawObject* self, PyObject* args, PyObject* kwds);
  static PyObject* shmpath(pyWriterClawObject* self, PyObject* args, PyObject* kwds);
  static PyObject* get_can_do_str(pyWriterClawObject* self, PyObject* args, PyObject* kwds);
};
#endif
