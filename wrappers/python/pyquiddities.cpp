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
 */

#include "./pyquiddities.hpp"
#include "./pyquiddity.hpp"

PyObject* pyQuiddities::tp_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
  pyQuidditiesObject* self = (pyQuidditiesObject*)type->tp_alloc(type, 0);
  return reinterpret_cast<PyObject*>(self);
}

int pyQuiddities::tp_init(pyQuidditiesObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyswitch = nullptr;

  if (!PyArg_ParseTuple(args, "O", &pyswitch)) return -1;
  self->pyswitch = pyswitch;

  return 0;
}

void pyQuiddities::tp_dealloc(pyQuidditiesObject* self) {
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* pyQuiddities::get_quiddities(pyQuidditiesObject* self) {
  auto quiddities = PyList_New(0);

  auto pyswitchobj = reinterpret_cast<pySwitch::pySwitchObject*>(self->pyswitch);
  auto switcher =  pyswitchobj->switcher.get();
  auto ids = switcher->quids<MPtr(&quiddity::Container::get_ids)>();

  PyObject* arg_list = nullptr;
  PyObject* capsule = nullptr;

  for (auto it = ids.begin(); it != ids.end(); ++it) {
    auto quid = switcher->quids<MPtr(&quiddity::Container::get_quiddity)>(*it);

    capsule = PyCapsule_New(static_cast<void*>(quid.get()), nullptr, nullptr);
    arg_list = Py_BuildValue("(O)", capsule);

    auto py_quiddity = PyObject_CallObject((PyObject*)&pyQuiddity::pyType, arg_list);
    PyList_Append(quiddities, py_quiddity);
  }

  Py_XDECREF(capsule);
  Py_XDECREF(arg_list);

  return quiddities;
}

PyObject* pyQuiddities::tp_descr_get(pyQuidditiesObject* self, PyObject* args, PyObject* kwds) {
  return get_quiddities(self);
}

Py_ssize_t pyQuiddities::sq_length(pyQuidditiesObject* self) {
  return PyList_Size(get_quiddities(self));
}

PyObject* pyQuiddities::sq_item(pyQuidditiesObject* self, Py_ssize_t i) {
  return PyList_GetItem(get_quiddities(self), i);
}

PySequenceMethods pyQuiddities::tp_as_sequence = {.sq_length = (lenfunc)sq_length,
                                                  .sq_item = (ssizeargfunc)sq_item };

PyTypeObject pyQuiddities::pyType = { PyVarObject_HEAD_INIT(nullptr, 0).tp_name = "Quiddities",
                                      .tp_basicsize = sizeof(pyQuidditiesObject),
                                      .tp_dealloc = (destructor)tp_dealloc,
                                      .tp_as_sequence = &tp_as_sequence,
                                      .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
                                      .tp_doc = ("A descriptor that queries all quiddities in the current session."),
                                      .tp_descr_get = (descrgetfunc)tp_descr_get,
                                      .tp_init = (initproc)tp_init,
                                      .tp_new = (newfunc) tp_new, };
