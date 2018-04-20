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

#include "./pyqrox.hpp"
#include "./pyquiddity.hpp"

PyMethodDef pyQrox::pyQrox_methods[] = {
    {"id", (PyCFunction)get_id, METH_NOARGS, "Provide identifiant for the quiddity."},
    {"name", (PyCFunction)get_name, METH_NOARGS, "Provide quiddity name."},
    {"quid", (PyCFunction)get_quid, METH_NOARGS, "Provide quiddity."},
    {nullptr}};

PyObject* pyQrox::Qrox_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyQroxObject* self;

  self = (pyQroxObject*)type->tp_alloc(type, 0);

  return (PyObject*)self;
}

int pyQrox::Qrox_init(pyQroxObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyqrox;
  static char* kwlist[] = {(char*)"qrox_c_ptr", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O", kwlist, &pyqrox)) return -1;

  auto* qrox = static_cast<quid::Qrox*>(PyCapsule_GetPointer(pyqrox, nullptr));

  self->qrox = std::make_unique<quid::Qrox>(*qrox);

  return 0;
}

void pyQrox::Qrox_dealloc(pyQroxObject* self) { Py_TYPE(self)->tp_free((PyObject*)self); }

PyTypeObject pyQrox::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyquid.Qrox", /* tp_name */
    sizeof(pyQroxObject),                                   /* tp_basicsize */
    0,                                                      /* tp_itemsize */
    (destructor)Qrox_dealloc,                               /* tp_dealloc */
    0,                                                      /* tp_print */
    0,                                                      /* tp_getattr */
    0,                                                      /* tp_setattr */
    0,                                                      /* tp_reserved */
    0,                                                      /* tp_repr */
    0,                                                      /* tp_as_number */
    0,                                                      /* tp_as_sequence */
    0,                                                      /* tp_as_mapping */
    0,                                                      /* tp_hash  */
    0,                                                      /* tp_call */
    0,                                                      /* tp_str */
    0,                                                      /* tp_getattro */
    0,                                                      /* tp_setattro */
    0,                                                      /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,               /* tp_flags */
    "pyQrox objects",                                       /* tp_doc */
    0,                                                      /* tp_traverse */
    0,                                                      /* tp_clear */
    0,                                                      /* tp_richcompare */
    0,                                                      /* tp_weaklistoffset */
    0,                                                      /* tp_iter */
    0,                                                      /* tp_iternext */
    pyQrox_methods,                                         /* tp_methods */
    0,                                                      /* tp_members */
    0,                                                      /* tp_getset */
    0,                                                      /* tp_base */
    0,                                                      /* tp_dict */
    0,                                                      /* tp_descr_get */
    0,                                                      /* tp_descr_set */
    0,                                                      /* tp_dictoffset */
    (initproc)Qrox_init,                                    /* tp_init */
    0,                                                      /* tp_alloc */
    Qrox_new                                                /* tp_new */
};

PyObject* pyQrox::get_id(pyQroxObject* self) { return PyLong_FromSize_t(self->qrox->get_id()); }

PyObject* pyQrox::get_name(pyQroxObject* self) {
  return PyUnicode_FromString(self->qrox->msg().c_str());
}

PyObject* pyQrox::get_quid(pyQroxObject* self) {
  auto qrox_capsule = PyCapsule_New(static_cast<void*>(self->qrox->get()), nullptr, nullptr);
  PyObject* argList = Py_BuildValue("(O)", qrox_capsule);
  PyObject* obj = PyObject_CallObject((PyObject*)&pyQuiddity::pyType, argList);
  Py_XDECREF(argList);
  Py_XDECREF(qrox_capsule);
  return obj;
}
