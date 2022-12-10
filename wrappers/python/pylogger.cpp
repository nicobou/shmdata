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

#include "./pylogger.hpp"

#include "pyswitch.hpp"
#include "switcher/switcher.hpp"

PyObject* pyLogger::tp_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
  pyLoggerObject* self;
  self = (pyLoggerObject*)type->tp_alloc(type, 0);
  return reinterpret_cast<PyObject*>(self);
}

int pyLogger::tp_init(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyswitch = nullptr;
  if (!PyArg_ParseTuple(args, "O", &pyswitch)) return -1;
  self->logger = reinterpret_cast<pySwitch::pySwitchObject*>(pyswitch)->switcher.get();
  if (!self->logger) return -1;
  return 0;
}

void pyLogger::tp_dealloc(pyLoggerObject* self) { Py_TYPE(self)->tp_free((PyObject*)self); }

PyObject* pyLogger::trace(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->sw_trace(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::debug(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->sw_debug(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::info(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->sw_info(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::warn(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->sw_warning(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::error(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->sw_error(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::critical(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->sw_critical(msg);
  Py_RETURN_TRUE;
}

PyMethodDef pyLogger::methods[] = {
    {"trace",
     (PyCFunction)pyLogger::trace,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `TRACE` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"debug",
     (PyCFunction)pyLogger::debug,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `DEBUG` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"info",
     (PyCFunction)pyLogger::info,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `INFO` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"warn",
     (PyCFunction)pyLogger::warn,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `WARN` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"error",
     (PyCFunction)pyLogger::error,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `ERROR` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"critical",
     (PyCFunction)pyLogger::critical,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `CRITICAL` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {nullptr}  // sentinel
};

PyTypeObject pyLogger::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0).tp_name = "pyLogger",
    .tp_basicsize = sizeof(pyLoggerObject),
    .tp_dealloc = (destructor)tp_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = "The `Logger` for Switcher.",
    .tp_methods = methods,
    .tp_init = (initproc)tp_init,
    .tp_new = (newfunc)tp_new,
};
