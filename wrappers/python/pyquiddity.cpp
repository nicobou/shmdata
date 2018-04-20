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

#include "./pyquiddity.hpp"
#include "switcher/scope-exit.hpp"

PyMethodDef pyQuiddity::pyQuiddity_methods[] = {
    {"set_str_str",
     (PyCFunction)pyQuiddity::set_str_str,
     METH_VARARGS | METH_KEYWORDS,
     "Set the property of a quiddity with a string value."},
    {"get_str_str",
     (PyCFunction)pyQuiddity::get_str_str,
     METH_VARARGS | METH_KEYWORDS,
     "Get the property of a quiddity."},
    {"invoke_str",
     (PyCFunction)pyQuiddity::invoke_str,
     METH_VARARGS | METH_KEYWORDS,
     "Get the property of a quiddity."},
    {"make_shmpath",
     (PyCFunction)pyQuiddity::make_shmpath,
     METH_VARARGS | METH_KEYWORDS,
     "Get shmpath of a quiddity, according to the given suffix."},
    {nullptr}};

PyObject* pyQuiddity::Quiddity_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyQuiddityObject* self;
  self = (pyQuiddityObject*)type->tp_alloc(type, 0);
  return (PyObject*)self;
}

int pyQuiddity::Quiddity_init(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyqrox;
  static char* kwlist[] = {(char*)"quid_c_ptr", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O", kwlist, &pyqrox)) return -1;
  auto* quid = static_cast<Quiddity*>(PyCapsule_GetPointer(pyqrox, nullptr));
  self->quid = quid;
  return 0;
}

void pyQuiddity::Quiddity_dealloc(pyQuiddityObject* self) {
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyTypeObject pyQuiddity::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyquid.Quiddity", /* tp_name */
    sizeof(pyQuiddityObject),                                   /* tp_basicsize */
    0,                                                          /* tp_itemsize */
    (destructor)Quiddity_dealloc,                               /* tp_dealloc */
    0,                                                          /* tp_print */
    0,                                                          /* tp_getattr */
    0,                                                          /* tp_setattr */
    0,                                                          /* tp_reserved */
    0,                                                          /* tp_repr */
    0,                                                          /* tp_as_number */
    0,                                                          /* tp_as_sequence */
    0,                                                          /* tp_as_mapping */
    0,                                                          /* tp_hash  */
    0,                                                          /* tp_call */
    0,                                                          /* tp_str */
    0,                                                          /* tp_getattro */
    0,                                                          /* tp_setattro */
    0,                                                          /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,                   /* tp_flags */
    "pyQuiddity objects",                                       /* tp_doc */
    0,                                                          /* tp_traverse */
    0,                                                          /* tp_clear */
    0,                                                          /* tp_richcompare */
    0,                                                          /* tp_weaklistoffset */
    0,                                                          /* tp_iter */
    0,                                                          /* tp_iternext */
    pyQuiddity_methods,                                         /* tp_methods */
    0,                                                          /* tp_members */
    0,                                                          /* tp_getset */
    0,                                                          /* tp_base */
    0,                                                          /* tp_dict */
    0,                                                          /* tp_descr_get */
    0,                                                          /* tp_descr_set */
    0,                                                          /* tp_dictoffset */
    (initproc)Quiddity_init,                                    /* tp_init */
    0,                                                          /* tp_alloc */
    Quiddity_new                                                /* tp_new */
};

PyObject* pyQuiddity::set_str_str(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* property = nullptr;
  const char* value = nullptr;
  static char* kwlist[] = {(char*)"property", (char*)"value", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "ss", kwlist, &property, &value)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  if (!self->quid->prop<MPtr(&PContainer::set_str_str)>(property, value)) return Py_False;
  Py_INCREF(Py_True);
  return Py_True;
}

PyObject* pyQuiddity::get_str_str(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* property = nullptr;
  static char* kwlist[] = {(char*)"property", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &property)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  return PyUnicode_FromString(self->quid->prop<MPtr(&PContainer::get_str_str)>(property).c_str());
}

PyObject* pyQuiddity::invoke_str(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* method = nullptr;
  PyObject* inv_args = nullptr;
  static char* kwlist[] = {(char*)"method", (char*)"args", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|O", kwlist, &method, &inv_args)) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  if (!PyList_Check(inv_args)) return nullptr;
  auto tuple_args = std::string();
  for (auto i = 0; i < PyList_Size(inv_args); ++i) {
    PyObject* item = nullptr;
    PyObject* item_str = nullptr;
    PyObject* repr = nullptr;
    item = PyList_GetItem(inv_args, i);
    if (!PyObject_TypeCheck(item, &PyUnicode_Type)) {
      repr = PyObject_Repr(item);
      item_str = PyUnicode_AsEncodedString(repr, "utf-8", "Error ");
      Py_XDECREF(repr);
    } else {
      item_str = PyUnicode_AsEncodedString(item, "utf-8", "Error ");
    }
    On_scope_exit {
      if (item_str) Py_XDECREF(item_str);
      if (repr) Py_XDECREF(repr);
    };

    if (tuple_args.empty())
      tuple_args = serialize::esc_for_tuple(PyBytes_AS_STRING(item_str));
    else
      tuple_args = tuple_args + "," + serialize::esc_for_tuple(PyBytes_AS_STRING(item_str));
  }

  auto res = self->quid->meth<MPtr(&MContainer::invoke_str)>(
      self->quid->meth<MPtr(&MContainer::get_id)>(method), tuple_args);
  if (!res) {
    return nullptr;
  }
  return PyUnicode_FromString(res.msg().c_str());
}

PyObject* pyQuiddity::make_shmpath(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* suffix = nullptr;
  static char* kwlist[] = {(char*)"suffix", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &suffix)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  return PyUnicode_FromString(self->quid->make_shmpath(suffix).c_str());
}
