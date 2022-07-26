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

#ifndef __SWITCHER_PYLOGGER_H__
#define __SWITCHER_PYLOGGER_H__

#include <Python.h>
#include <spdlog/spdlog.h>

#include <memory>

class pyLogger {
 public:
  using pyLoggerObject = struct { PyObject_HEAD std::shared_ptr<spdlog::logger> logger; };
  static PyMethodDef methods[];
  static PyTypeObject pyType;

 private:
  static PyObject* tp_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
  static int tp_init(pyLoggerObject* self, PyObject* args, PyObject* kwds);
  static void tp_dealloc(pyLoggerObject* self);

  // binding methods
  static PyObject* trace(pyLoggerObject* self, PyObject* args, PyObject* kwds);
  static PyObject* debug(pyLoggerObject* self, PyObject* args, PyObject* kwds);
  static PyObject* info(pyLoggerObject* self, PyObject* args, PyObject* kwds);
  static PyObject* warn(pyLoggerObject* self, PyObject* args, PyObject* kwds);
  static PyObject* error(pyLoggerObject* self, PyObject* args, PyObject* kwds);
  static PyObject* critical(pyLoggerObject* self, PyObject* args, PyObject* kwds);
};

#endif