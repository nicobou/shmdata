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

#include "./pyswitch.hpp"
#include "./pyqrox.hpp"
#include "switcher/console-logger.hpp"
#include "switcher/silent-logger.hpp"

PyMethodDef pySwitch::pySwitch_methods[] = {
    {"name", (PyCFunction)pySwitch::get_name, METH_NOARGS, "Provide switcher name."},
    {"create", (PyCFunction)pySwitch::create, METH_VARARGS | METH_KEYWORDS, "Create a quiddity."},
    {nullptr}};

PyTypeObject pySwitch::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyquid.Switcher", /* tp_name */
    sizeof(pySwitchObject),                                     /* tp_basicsize */
    0,                                                          /* tp_itemsize */
    (destructor)Switcher_dealloc,                               /* tp_dealloc */
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
    "Switcher objects",                                         /* tp_doc */
    0,                                                          /* tp_traverse */
    0,                                                          /* tp_clear */
    0,                                                          /* tp_richcompare */
    0,                                                          /* tp_weaklistoffset */
    0,                                                          /* tp_iter */
    0,                                                          /* tp_iternext */
    pySwitch_methods,                                           /* tp_methods */
    0,                                                          /* tp_members */
    0,                                                          /* tp_getset */
    0,                                                          /* tp_base */
    0,                                                          /* tp_dict */
    0,                                                          /* tp_descr_get */
    0,                                                          /* tp_descr_set */
    0,                                                          /* tp_dictoffset */
    (initproc)Switcher_init,                                    /* tp_init */
    0,                                                          /* tp_alloc */
    Switcher_new                                                /* tp_new */
};

PyObject* pySwitch::Switcher_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pySwitchObject* self;

  self = (pySwitchObject*)type->tp_alloc(type, 0);
  if (self != nullptr) {
    self->name = PyUnicode_FromString("default");
    if (self->name == nullptr) {
      Py_XDECREF(self);
      return nullptr;
    }
  }

  return (PyObject*)self;
}

int pySwitch::Switcher_init(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  PyObject* name = nullptr;
  PyObject* showDebug = nullptr;
  PyObject* tmp = nullptr;

  static char* kwlist[] = {(char*)"name", (char*)"debug", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|O", kwlist, &name, &showDebug)) return -1;

  tmp = self->name;
  Py_INCREF(name);
  self->name = name;
  Py_XDECREF(tmp);

  if (showDebug && PyBool_Check(showDebug)) {
    self->switcher = Switcher::make_switcher<ConsoleLogger>(PyUnicode_AsUTF8(self->name));
  } else {
    self->switcher = Switcher::make_switcher<SilentLogger>(PyUnicode_AsUTF8(self->name));
  }

  if (!self->switcher) {
    return -1;
  }

  self->switcher->factory<MPtr(&quid::Factory::scan_dir)>(
      self->switcher->factory<MPtr(&quid::Factory::get_default_plugin_dir)>());

  return 0;
}

void pySwitch::Switcher_dealloc(pySwitchObject* self) {
  Py_XDECREF(self->name);
  Switcher::ptr empty;
  self->switcher.swap(empty);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* pySwitch::get_name(pySwitchObject* self) {
  return PyUnicode_FromString(self->switcher.get()->get_name().c_str());
}

PyObject* pySwitch::create(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* type = nullptr;
  const char* name = nullptr;
  static char* kwlist[] = {(char*)"type", (char*)"name", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|s", kwlist, &type, &name)) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  auto qrox =
      self->switcher->quids<MPtr(&quid::Container::create)>(type, name ? name : std::string());
  if (!qrox) return nullptr;

  auto qrox_capsule = PyCapsule_New(static_cast<void*>(&qrox), nullptr, nullptr);
  PyObject* argList = Py_BuildValue("(O)", qrox_capsule);
  PyObject* obj = PyObject_CallObject((PyObject*)&pyQrox::pyType, argList);
  Py_XDECREF(argList);
  Py_XDECREF(qrox_capsule);
  return obj;
}
