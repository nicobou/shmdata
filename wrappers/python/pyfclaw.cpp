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

#include "./pyfclaw.hpp"

#include <shmdata/type.hpp>
#include <switcher/utils/scope-exit.hpp>

#include "./pyquiddity.hpp"
#include "./pyswitch.hpp"
#include "./pywclaw.hpp"
#include "./ungiled.hpp"

PyObject* pyFollowerClaw::FollowerClaw_new(PyTypeObject* type,
                                           PyObject* /*args*/,
                                           PyObject* /*kwds*/) {
  pyFollowerClawObject* self;
  self = (pyFollowerClawObject*)type->tp_alloc(type, 0);
  return (PyObject*)self;
}

int pyFollowerClaw::FollowerClaw_init(pyFollowerClawObject* self, PyObject* args, PyObject* kwds) {
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

  Quiddity* raw_quid;
  if (quid_c_ptr) {
    raw_quid = static_cast<Quiddity*>(PyCapsule_GetPointer(quid_c_ptr, nullptr));
  } else {
    raw_quid = reinterpret_cast<pyQuiddity::pyQuiddityObject*>(quid)->quid.lock().get();
  }

  self->id = raw_quid->claw<MPtr(&Claw::get_sfid)>(label);
  self->quid = raw_quid->get_weak_ptr_to_this();
  if (Ids::kInvalid == self->id) {
    PyErr_Format(PyExc_RuntimeError, "error follower label not found for in Quiddity Claw");
    return -1;
  }
  return 0;
}

void pyFollowerClaw::FollowerClaw_dealloc(pyFollowerClawObject* self) {
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyDoc_STRVAR(pyfollowerclaw_label_doc,
             "Get the label of the follower Claw.\n"
             "Arguments: ()\n"
             "Returns: the label (string)\n");

PyObject* pyFollowerClaw::label(pyFollowerClawObject* self, PyObject* args, PyObject* kwds) {
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  return PyUnicode_FromString(quid->claw<MPtr(&Claw::get_follower_label)>(self->id).c_str());
}

PyDoc_STRVAR(pyfollowerclaw_get_can_do_str_doc,
             "Get the list of shmdata types produced by the follower.\n"
             "Arguments: ()\n"
             "Returns: a list of shmdata types (list of string)\n");

PyObject* pyFollowerClaw::get_can_do_str(pyFollowerClawObject* self,
                                         PyObject* args,
                                         PyObject* kwds) {
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  auto can_dos = quid->claw<MPtr(&Claw::get_follower_can_do)>(self->id);
  PyObject* res = PyList_New(can_dos.size());
  int i = 0;
  for (const auto& can_do : can_dos) {
    PyList_SetItem(res, i, PyUnicode_FromString(can_do.str().c_str()));
    ++i;
  }
  return res;
}

PyDoc_STRVAR(pyfollowerclaw_id_doc,
             "Get the identifier (swid) of the follower Claw.\n"
             "Arguments: ()\n"
             "Returns: the id (long)\n");

PyObject* pyFollowerClaw::id(pyFollowerClawObject* self, PyObject* args, PyObject* kwds) {
  return PyLong_FromSize_t(self->id);
}

PyDoc_STRVAR(pyfollowerclaw_connect_doc,
             "Connect to a WriterClaw of an other quiddity.\n"
             "Arguments: (quid, wclaw)\n"
             "Returns: a new follower claw if self is a meta follower, or self (FollowerClaw)\n");

PyObject* pyFollowerClaw::connect(pyFollowerClawObject* self, PyObject* args, PyObject* kwds) {
  PyObject* connect_quid_arg = nullptr;
  PyObject* wclaw = nullptr;
  static char* kwlist[] = {(char*)"quid", (char*)"wclaw", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &connect_quid_arg, &wclaw)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (connect_quid_arg &&
      !PyObject_IsInstance(connect_quid_arg, reinterpret_cast<PyObject*>(&pyQuiddity::pyType))) {
    PyErr_SetString(PyExc_TypeError, "error quid argument is not a pyquid.Quiddidy instance");
    return nullptr;
  }
  auto connect_quid =
      reinterpret_cast<pyQuiddity::pyQuiddityObject*>(connect_quid_arg)->quid.lock();
  if (!connect_quid) return nullptr;

  if (wclaw && !PyObject_IsInstance(wclaw, reinterpret_cast<PyObject*>(&pyWriterClaw::pyType))) {
    PyErr_SetString(PyExc_TypeError, "error wclaw argument is not a pyquid.WriterClaw instance");
    return nullptr;
  }

  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  const auto res = quid->claw<MPtr(&Claw::connect)>(
      self->id,
      connect_quid->get_id(),
      reinterpret_cast<pyWriterClaw::pyWriterClawObject*>(wclaw)->id);
  if (Ids::kInvalid == res) {
    PyErr_Format(PyExc_RuntimeError, "failed to connect, check switcher log for more information");
    return nullptr;
  }

  if (res == self->id) {
    Py_INCREF(self);
    return reinterpret_cast<PyObject*>(self);
  }
  PyObject* empty_tuple = PyTuple_New(0);
  On_scope_exit { Py_XDECREF(empty_tuple); };
  PyObject* keyworded_args = PyDict_New();
  On_scope_exit { Py_XDECREF(keyworded_args); };
  Py_INCREF(Py_None);
  PyDict_SetItemString(keyworded_args, "quid", Py_None);
  PyDict_SetItemString(
      keyworded_args,
      "label",
      PyUnicode_FromString(quid->claw<MPtr(&Claw::get_follower_label)>(res).c_str()));
  auto quid_capsule = PyCapsule_New(static_cast<void*>(quid.get()), nullptr, nullptr);
  On_scope_exit { Py_XDECREF(quid_capsule); };
  PyDict_SetItemString(keyworded_args, "quid_c_ptr", quid_capsule);
  return PyObject_Call((PyObject*)&pyFollowerClaw::pyType, empty_tuple, keyworded_args);
}

PyDoc_STRVAR(pyfollowerclaw_connect_quid_doc,
             "Connect to the first compatible Writer Claw of an other quiddity.\n"
             "Arguments: (quid)\n"
             "Returns: a new follower claw if self is a meta follower, or self (FollowerClaw)\n");

PyObject* pyFollowerClaw::connect_quid(pyFollowerClawObject* self, PyObject* args, PyObject* kwds) {
  PyObject* connect_quid_arg = nullptr;
  static char* kwlist[] = {(char*)"quid", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O", kwlist, &connect_quid_arg)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (connect_quid_arg &&
      !PyObject_IsInstance(connect_quid_arg, reinterpret_cast<PyObject*>(&pyQuiddity::pyType))) {
    PyErr_SetString(PyExc_TypeError, "error quid argument is not a pyquid.Quiddidy instance");
    return nullptr;
  }
  auto connect_quid =
      reinterpret_cast<pyQuiddity::pyQuiddityObject*>(connect_quid_arg)->quid.lock();
  if (!connect_quid) return nullptr;

  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  const auto res = quid->claw<MPtr(&Claw::connect_quid)>(self->id, connect_quid->get_id());
  if (Ids::kInvalid == res) {
    PyErr_Format(PyExc_RuntimeError, "failed to connect, check switcher log for more information");
    return nullptr;
  }

  if (res == self->id) {
    Py_INCREF(self);
    return reinterpret_cast<PyObject*>(self);
  }
  PyObject* empty_tuple = PyTuple_New(0);
  On_scope_exit { Py_XDECREF(empty_tuple); };
  PyObject* keyworded_args = PyDict_New();
  On_scope_exit { Py_XDECREF(keyworded_args); };
  Py_INCREF(Py_None);
  PyDict_SetItemString(keyworded_args, "quid", Py_None);
  PyDict_SetItemString(
      keyworded_args,
      "label",
      PyUnicode_FromString(quid->claw<MPtr(&Claw::get_follower_label)>(res).c_str()));
  auto quid_capsule = PyCapsule_New(static_cast<void*>(quid.get()), nullptr, nullptr);
  On_scope_exit { Py_XDECREF(quid_capsule); };
  PyDict_SetItemString(keyworded_args, "quid_c_ptr", quid_capsule);
  return PyObject_Call((PyObject*)&pyFollowerClaw::pyType, empty_tuple, keyworded_args);
}

PyDoc_STRVAR(pyfollowerclaw_connect_raw_doc,
             "Connect to a raw shmdata.\n"
             "Arguments: (quiddity, shmpath)\n"
             "Returns: a new follower claw if self is a meta follower, or self (FollowerClaw)\n");

PyObject* pyFollowerClaw::connect_raw(pyFollowerClawObject* self, PyObject* args, PyObject* kwds) {
  char* shmpath = nullptr;
  static char* kwlist[] = {(char*)"shmpath", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &shmpath)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }

  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  const auto res = quid->claw<MPtr(&Claw::connect_raw)>(self->id, std::string(shmpath));
  if (Ids::kInvalid == res) {
    PyErr_Format(PyExc_RuntimeError, "failed to connect, check switcher log for more information");
    return nullptr;
  }

  if (res == self->id) {
    Py_INCREF(self);
    return reinterpret_cast<PyObject*>(self);
  }
  PyObject* empty_tuple = PyTuple_New(0);
  On_scope_exit { Py_XDECREF(empty_tuple); };
  PyObject* keyworded_args = PyDict_New();
  On_scope_exit { Py_XDECREF(keyworded_args); };
  Py_INCREF(Py_None);
  PyDict_SetItemString(keyworded_args, "quid", Py_None);
  PyDict_SetItemString(
      keyworded_args,
      "label",
      PyUnicode_FromString(quid->claw<MPtr(&Claw::get_follower_label)>(res).c_str()));
  auto quid_capsule = PyCapsule_New(static_cast<void*>(quid.get()), nullptr, nullptr);
  On_scope_exit { Py_XDECREF(quid_capsule); };
  PyDict_SetItemString(keyworded_args, "quid_c_ptr", quid_capsule);
  return PyObject_Call((PyObject*)&pyFollowerClaw::pyType, empty_tuple, keyworded_args);
}

PyDoc_STRVAR(pyfollowerclaw_disconnect_doc,
             "Disconnect the follower Claw.\n"
             "Arguments: ()\n"
             "Returns: success (Bool)\n");

PyObject* pyFollowerClaw::disconnect(pyFollowerClawObject* self, PyObject* args, PyObject* kwds) {
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  if (!quid->claw<MPtr(&Claw::disconnect)>(self->id)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyfollowerclaw_can_do_shmtype_str_doc,
             "Test is the string-formatted shmtype is compatible for connection.\n"
             "Arguments: (shmtype)\n"
             "Returns: True of False (Bool)\n");

PyObject* pyFollowerClaw::can_do_shmtype_str(pyFollowerClawObject* self,
                                             PyObject* args,
                                             PyObject* kwds) {
  char* shmtype = nullptr;
  static char* kwlist[] = {(char*)"shmtype", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &shmtype)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  if (!quid->claw<MPtr(&Claw::sfid_can_do_shmtype)>(self->id,
                                                    ::shmdata::Type(std::string(shmtype)))) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyfollowerclaw_can_do_writer_claw_doc,
             "Test if the a writer claw id compatible for connection.\n"
             "Arguments: (quid, wclaw)\n"
             "Returns: True of False (Boolean)\n");

PyObject* pyFollowerClaw::can_do_writer_claw(pyFollowerClawObject* self,
                                             PyObject* args,
                                             PyObject* kwds) {
  PyObject* can_do_quid_arg = nullptr;
  PyObject* wclaw = nullptr;
  static char* kwlist[] = {(char*)"quid", (char*)"wclaw", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &can_do_quid_arg, &wclaw)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (can_do_quid_arg &&
      !PyObject_IsInstance(can_do_quid_arg, reinterpret_cast<PyObject*>(&pyQuiddity::pyType))) {
    PyErr_SetString(PyExc_TypeError, "error quid argument is not a pyquid.Quiddidy instance");
    return nullptr;
  }
  auto can_do_quid = reinterpret_cast<pyQuiddity::pyQuiddityObject*>(can_do_quid_arg)->quid.lock();
  if (!can_do_quid) return nullptr;

  if (wclaw && !PyObject_IsInstance(wclaw, reinterpret_cast<PyObject*>(&pyWriterClaw::pyType))) {
    PyErr_SetString(PyExc_TypeError, "error wclaw argument is not a pyquid.WriterClaw instance");
    return nullptr;
  }
  auto quid = self->quid.lock();
  if (!quid) {
    PyErr_SetString(PyExc_MemoryError, "Quiddity or parent Switcher has been deleted");
    return nullptr;
  }
  if (!quid->claw<MPtr(&Claw::can_do_swid)>(
          self->id,
          can_do_quid->get_id(),
          reinterpret_cast<pyWriterClaw::pyWriterClawObject*>(wclaw)->id)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyMethodDef pyFollowerClaw::pyFollowerClaw_methods[] = {
    {"label", (PyCFunction)pyFollowerClaw::label, METH_NOARGS, pyfollowerclaw_label_doc},
    {"id", (PyCFunction)pyFollowerClaw::id, METH_NOARGS, pyfollowerclaw_id_doc},
    {"get_can_do_str",
     (PyCFunction)pyFollowerClaw::get_can_do_str,
     METH_VARARGS | METH_KEYWORDS,
     pyfollowerclaw_get_can_do_str_doc},
    {"can_do_shmtype_str",
     (PyCFunction)pyFollowerClaw::can_do_shmtype_str,
     METH_VARARGS | METH_KEYWORDS,
     pyfollowerclaw_can_do_shmtype_str_doc},
    {"can_do_writer_claw",
     (PyCFunction)pyFollowerClaw::can_do_writer_claw,
     METH_VARARGS | METH_KEYWORDS,
     pyfollowerclaw_can_do_writer_claw_doc},
    {"connect",
     (PyCFunction)pyFollowerClaw::connect,
     METH_VARARGS | METH_KEYWORDS,
     pyfollowerclaw_connect_doc},
    {"connect_quid",
     (PyCFunction)pyFollowerClaw::connect_quid,
     METH_VARARGS | METH_KEYWORDS,
     pyfollowerclaw_connect_quid_doc},
    {"connect_raw",
     (PyCFunction)pyFollowerClaw::connect_raw,
     METH_VARARGS | METH_KEYWORDS,
     pyfollowerclaw_connect_raw_doc},
    {"disconnect",
     (PyCFunction)pyFollowerClaw::disconnect,
     METH_NOARGS,
     pyfollowerclaw_disconnect_doc},
    {nullptr}};

PyDoc_STRVAR(pyquid_pyfollowerclaw_doc,
             "FollowerClaw objects.\n"
             "A FollowerClaw handles a single Shmdata Follower in a Quiddity\n");

PyTypeObject pyFollowerClaw::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyquid.FollowerClaw", /* tp_name */
    sizeof(pyFollowerClawObject),                                   /* tp_basicsize */
    0,                                                              /* tp_itemsize */
    (destructor)FollowerClaw_dealloc,                               /* tp_dealloc */
    0,                                                              /* tp_print */
    0,                                                              /* tp_getattr */
    0,                                                              /* tp_setattr */
    0,                                                              /* tp_reserved */
    0,                                                              /* tp_repr */
    0,                                                              /* tp_as_number */
    0,                                                              /* tp_as_sequence */
    0,                                                              /* tp_as_mapping */
    0,                                                              /* tp_hash  */
    0,                                                              /* tp_call */
    0,                                                              /* tp_str */
    0,                                                              /* tp_getattro */
    0,                                                              /* tp_setattro */
    0,                                                              /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,                       /* tp_flags */
    pyquid_pyfollowerclaw_doc,                                      /* tp_doc */
    0,                                                              /* tp_traverse */
    0,                                                              /* tp_clear */
    0,                                                              /* tp_richcompare */
    0,                                                              /* tp_weaklistoffset */
    0,                                                              /* tp_iter */
    0,                                                              /* tp_iternext */
    pyFollowerClaw_methods,                                         /* tp_methods */
    0,                                                              /* tp_members */
    0,                                                              /* tp_getset */
    0,                                                              /* tp_base */
    0,                                                              /* tp_dict */
    0,                                                              /* tp_descr_get */
    0,                                                              /* tp_descr_set */
    0,                                                              /* tp_dictoffset */
    (initproc)FollowerClaw_init,                                    /* tp_init */
    0,                                                              /* tp_alloc */
    FollowerClaw_new                                                /* tp_new */
};
