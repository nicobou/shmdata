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

#include "./pywclaw.hpp"

#include "./pyquiddity.hpp"

PyObject* pyWriterClaw::WriterClaw_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyWriterClawObject* self;
  self = (pyWriterClawObject*)type->tp_alloc(type, 0);
  return (PyObject*)self;
}

int pyWriterClaw::WriterClaw_init(pyWriterClawObject* self, PyObject* args, PyObject* kwds) {
  PyObject* quid = nullptr;
  char* label = nullptr;
  PyObject* quid_c_ptr = nullptr;
  static char* kwlist[] = {(char*)"quid", (char*)"label", (char*)"quid_c_ptr", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "Os|O", kwlist, &quid, &label, &quid_c_ptr)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return -1;
  }
  if (!quid_c_ptr && quid &&
      !PyObject_IsInstance(quid, reinterpret_cast<PyObject*>(&pyQuiddity::pyType))) {
    PyErr_SetString(PyExc_TypeError, "error quid argument is not an instance of a pyquid.Quiddity");
    return -1;
  }

  Quiddity* raw_quid = nullptr;
  if (quid_c_ptr) {
    raw_quid = static_cast<Quiddity*>(PyCapsule_GetPointer(quid_c_ptr, nullptr));
  } else {
    raw_quid = reinterpret_cast<pyQuiddity::pyQuiddityObject*>(quid)->quid.lock().get();
  }
  self->id = raw_quid->claw<MPtr(&Claw::get_swid)>(label);
  self->quid = raw_quid->get_weak_ptr_to_this();
  if (Ids::kInvalid == self->id) {
    PyErr_Format(PyExc_RuntimeError, "error label not found for writer in Quiddity Claw");
    return -1;
  }
  return 0;
}

void pyWriterClaw::WriterClaw_dealloc(pyWriterClawObject* self) {
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyDoc_STRVAR(pywriterclaw_label_doc,
             "Get the label of the writer Claw.\n"
             "Arguments: ()\n"
             "Returns: the label (string)\n");

PyObject* pyWriterClaw::label(pyWriterClawObject* self, PyObject* args, PyObject* kwds) {
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  return PyUnicode_FromString(quid->claw<MPtr(&Claw::get_writer_label)>(self->id).c_str());
}

PyDoc_STRVAR(pywriterclaw_shmpath_doc,
             "Get the shmdata path of the writer Claw.\n"
             "Arguments: ()\n"
             "Returns: the shmpath (string)\n");

PyObject* pyWriterClaw::shmpath(pyWriterClawObject* self, PyObject* args, PyObject* kwds) {
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  return PyUnicode_FromString(quid->claw<MPtr(&Claw::get_writer_shmpath)>(self->id).c_str());
}

PyDoc_STRVAR(pywriterclaw_get_can_do_str_doc,
             "Get the list of shmdata types produced by the writer.\n"
             "Arguments: ()\n"
             "Returns: a list of shmdata types (list of string)\n");

PyObject* pyWriterClaw::get_can_do_str(pyWriterClawObject* self, PyObject* args, PyObject* kwds) {
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  auto can_dos = quid->claw<MPtr(&Claw::get_writer_can_do)>(self->id);
  PyObject* res = PyList_New(can_dos.size());
  int i = 0;
  for (const auto& can_do : can_dos) {
    PyList_SetItem(res, i, PyUnicode_FromString(can_do.str().c_str()));
    ++i;
  }
  return res;
}

PyDoc_STRVAR(pywriterclaw_id_doc,
             "Get the identifier (swid) of the writer Claw.\n"
             "Arguments: ()\n"
             "Returns: the id (long)\n");

PyObject* pyWriterClaw::id(pyWriterClawObject* self, PyObject* args, PyObject* kwds) {
  return PyLong_FromSize_t(self->id);
}

PyMethodDef pyWriterClaw::pyWriterClaw_methods[] = {
    {"label", (PyCFunction)pyWriterClaw::label, METH_NOARGS, pywriterclaw_label_doc},
    {"shmpath", (PyCFunction)pyWriterClaw::shmpath, METH_NOARGS, pywriterclaw_shmpath_doc},
    {"id", (PyCFunction)pyWriterClaw::id, METH_NOARGS, pywriterclaw_id_doc},
    {"get_can_do_str",
     (PyCFunction)pyWriterClaw::get_can_do_str,
     METH_NOARGS,
     pywriterclaw_get_can_do_str_doc},
    {nullptr}};

PyDoc_STRVAR(pyquid_pywriterclaw_doc,
             "WriterClaw objects.\n"
             "A WriterClaw handles a single Shmdata writer in a Quiddity\n");

PyTypeObject pyWriterClaw::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyquid.WriterClaw", /* tp_name */
    sizeof(pyWriterClawObject),                                   /* tp_basicsize */
    0,                                                            /* tp_itemsize */
    (destructor)WriterClaw_dealloc,                               /* tp_dealloc */
    0,                                                            /* tp_print */
    0,                                                            /* tp_getattr */
    0,                                                            /* tp_setattr */
    0,                                                            /* tp_reserved */
    0,                                                            /* tp_repr */
    0,                                                            /* tp_as_number */
    0,                                                            /* tp_as_sequence */
    0,                                                            /* tp_as_mapping */
    0,                                                            /* tp_hash  */
    0,                                                            /* tp_call */
    0,                                                            /* tp_str */
    0,                                                            /* tp_getattro */
    0,                                                            /* tp_setattro */
    0,                                                            /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,                     /* tp_flags */
    pyquid_pywriterclaw_doc,                                      /* tp_doc */
    0,                                                            /* tp_traverse */
    0,                                                            /* tp_clear */
    0,                                                            /* tp_richcompare */
    0,                                                            /* tp_weaklistoffset */
    0,                                                            /* tp_iter */
    0,                                                            /* tp_iternext */
    pyWriterClaw_methods,                                         /* tp_methods */
    0,                                                            /* tp_members */
    0,                                                            /* tp_getset */
    0,                                                            /* tp_base */
    0,                                                            /* tp_dict */
    0,                                                            /* tp_descr_get */
    0,                                                            /* tp_descr_set */
    0,                                                            /* tp_dictoffset */
    (initproc)WriterClaw_init,                                    /* tp_init */
    0,                                                            /* tp_alloc */
    WriterClaw_new                                                /* tp_new */
};
