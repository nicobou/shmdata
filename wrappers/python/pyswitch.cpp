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
#include <switcher/console-logger.hpp>
#include <switcher/silent-logger.hpp>
#include "./pyinfotree.hpp"
#include "./pyqrox.hpp"

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

PyDoc_STRVAR(pyswitch_name_doc,
             "Provide switcher instance name.\n"
             "Arguments: None\n"
             "Returns: the name (string)\n"
             "Get the name provided to the switcher instance at creation.");

PyObject* pySwitch::name(pySwitchObject* self) {
  return PyUnicode_FromString(self->switcher.get()->get_name().c_str());
}

PyDoc_STRVAR(pyswitch_version_doc,
             "Get switcher version.\n"
             "Arguments: None\n"
             "Returns: the libswitcher version (string)\n");

PyObject* pySwitch::version(pySwitchObject* self) {
  return PyUnicode_FromString(self->switcher.get()->get_switcher_version().c_str());
}

PyDoc_STRVAR(pyswitch_create_doc,
             "Create a quiddity.\n"
             "Arguments: (type, name)\n"
             "Returns: a handle to the created quiddity (pyquid.Qrox), or None\n");

PyObject* pySwitch::create(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* type = nullptr;
  const char* name = nullptr;
  static char* kwlist[] = {(char*)"type", (char*)"name", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|s", kwlist, &type, &name)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto qrox =
      self->switcher->quids<MPtr(&quid::Container::create)>(type, name ? name : std::string());
  if (!qrox) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto qrox_capsule = PyCapsule_New(static_cast<void*>(&qrox), nullptr, nullptr);
  PyObject* argList = Py_BuildValue("(O)", qrox_capsule);
  PyObject* obj = PyObject_CallObject((PyObject*)&pyQrox::pyType, argList);
  Py_XDECREF(argList);
  Py_XDECREF(qrox_capsule);
  return obj;
}

PyDoc_STRVAR(pyswitch_remove_doc,
             "Remove a quiddity.\n"
             "Arguments: (name)\n"
             "Returns: true or false\n");

PyObject* pySwitch::remove(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  long int id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "l", kwlist, &id)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  if (!self->switcher->quids<MPtr(&quid::Container::remove)>(id)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyswitch_get_qrox_from_name_doc,
             "Get a Qrox of an existing quiddity (from its name).\n"
             "Arguments: (name)\n"
             "Returns: a handle to the quiddity (pyquid.Qrox), or None.\n");

PyObject* pySwitch::get_qrox_from_name(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* name = nullptr;
  static char* kwlist[] = {(char*)"name", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &name)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto qrox = self->switcher->quids<MPtr(&quid::Container::get_qrox_from_name)>(name);
  if (!qrox) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto qrox_capsule = PyCapsule_New(static_cast<void*>(&qrox), nullptr, nullptr);
  PyObject* argList = Py_BuildValue("(O)", qrox_capsule);
  PyObject* obj = PyObject_CallObject((PyObject*)&pyQrox::pyType, argList);
  Py_XDECREF(argList);
  Py_XDECREF(qrox_capsule);
  return obj;
}

PyDoc_STRVAR(pyswitch_get_qrox_doc,
             "Get a Qrox of an existing quiddity.\n"
             "Arguments: (id)\n"
             "Returns: a handle to the quiddity (pyquid.Qrox), or None.\n");

PyObject* pySwitch::get_qrox(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  long int id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "l", kwlist, &id)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto qrox = self->switcher->quids<MPtr(&quid::Container::get_qrox)>(id);
  if (!qrox) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto qrox_capsule = PyCapsule_New(static_cast<void*>(&qrox), nullptr, nullptr);
  PyObject* argList = Py_BuildValue("(O)", qrox_capsule);
  PyObject* obj = PyObject_CallObject((PyObject*)&pyQrox::pyType, argList);
  Py_XDECREF(argList);
  Py_XDECREF(qrox_capsule);
  return obj;
}

PyDoc_STRVAR(pyswitch_get_state_doc,
             "Get the current state of the quiddities.\n"
             "Arguments: None\n"
             "Returns: an InfoTree object containing state description.\n");

PyObject* pySwitch::get_state(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto tree = self->switcher->get_state();
  return pyInfoTree::make_pyobject_from_c_ptr(tree.get(), true);
}

PyDoc_STRVAR(pyswitch_reset_state_doc,
             "Reset initial state for saving. When loading a new state, quiddities "
             "created after a call to reset_state will be cleared.\n The clear "
             "argument removes all created quiddities since the last call to reset_state "
             "(clear is true by default).\n"
             "Arguments: clear (bool)\n"
             "Returns: None.\n");

PyObject* pySwitch::reset_state(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  int clear = 1;
  static char* kwlist[] = {(char*)"clear", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|p", kwlist, &clear)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  self->switcher->reset_state(clear ? true : false);
  Py_INCREF(Py_None);
  return Py_None;
}

PyDoc_STRVAR(pyswitch_load_state_doc,
             "Load a switcher state. \n"
             "Arguments: state (InfoTree)\n"
             "Returns: True or False.\n");

PyObject* pySwitch::load_state(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyinfotree;
  static char* kwlist[] = {(char*)"state", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O", kwlist, &pyinfotree)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (!PyObject_IsInstance(pyinfotree, reinterpret_cast<PyObject*>(&pyInfoTree::pyType))) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (!self->switcher->load_state(
          reinterpret_cast<pyInfoTree::pyInfoTreeObject*>(pyinfotree)->tree)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyswitch_list_classes_doc,
             "Get the list of available type of quiddity.\n"
             "Arguments: (None)\n"
             "Returns: A list a of class name.\n");

PyObject* pySwitch::list_classes(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto class_list = self->switcher->factory<MPtr(&quid::Factory::get_class_list)>();
  PyObject* result = PyList_New(class_list.size());
  for (unsigned int i = 0; i < class_list.size(); ++i) {
    PyList_SetItem(result, i, Py_BuildValue("s", class_list[i].c_str()));
  }
  return result;
}

PyDoc_STRVAR(pyswitch_classes_doc_doc,
             "Get a JSON documentation of all classes.\n"
             "Arguments: (None)\n"
             "Returns: JSON-formated documentation of all classes available.\n");

PyObject* pySwitch::classes_doc(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  return PyUnicode_FromString(
      JSONSerializer::serialize(
          self->switcher->factory<MPtr(&quid::Factory::get_classes_doc)>().get())
          .c_str());
}

PyDoc_STRVAR(pyswitch_class_doc_doc,
             "Get a JSON documentation of a given classes.\n"
             "Arguments: (class)\n"
             "Returns: JSON-formated documentation of the class.\n");

PyObject* pySwitch::class_doc(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* class_name = nullptr;
  static char* kwlist[] = {(char*)"class", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &class_name)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  return PyUnicode_FromString(
      JSONSerializer::serialize(self->switcher->factory<MPtr(&quid::Factory::get_classes_doc)>()
                                    ->get_tree(std::string(".classes.") + class_name)
                                    .get())
          .c_str());
}

PyDoc_STRVAR(pyswitch_list_quids_doc,
             "Get the list of instanciated quiddities.\n"
             "Arguments: (None)\n"
             "Returns: A list a of quiddities.\n");

PyObject* pySwitch::list_quids(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto quids = self->switcher->quids<MPtr(&quid::Container::get_names)>();
  PyObject* result = PyList_New(quids.size());
  for (unsigned int i = 0; i < quids.size(); ++i) {
    PyList_SetItem(result, i, Py_BuildValue("s", quids[i].c_str()));
  }
  return result;
}

PyDoc_STRVAR(pyswitch_quids_descr_doc,
             "Get a JSON description of all the instanciated quiddities.\n"
             "Arguments: (None)\n"
             "Returns: the JSON-formated description.\n");

PyObject* pySwitch::quids_descr(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  return PyUnicode_FromString(
      JSONSerializer::serialize(
          self->switcher->quids<MPtr(&quid::Container::get_quiddities_description)>().get())
          .c_str());
}

PyDoc_STRVAR(pyswitch_quid_descr_doc,
             "Get a JSON description of a given quiddity.\n"
             "Arguments: (id)\n"
             "Returns: JSON-formated description of the quiddity.\n");

PyObject* pySwitch::quid_descr(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  int id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "i", kwlist, &id) && 0 != id) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  return PyUnicode_FromString(
      JSONSerializer::serialize(
          self->switcher->quids<MPtr(&quid::Container::get_quiddity_description)>(id).get())
          .c_str());
}

PyMethodDef pySwitch::pySwitch_methods[] = {
    {"name", (PyCFunction)pySwitch::name, METH_NOARGS, pyswitch_name_doc},
    {"version", (PyCFunction)pySwitch::version, METH_NOARGS, pyswitch_version_doc},
    {"create", (PyCFunction)pySwitch::create, METH_VARARGS | METH_KEYWORDS, pyswitch_create_doc},
    {"remove", (PyCFunction)pySwitch::remove, METH_VARARGS | METH_KEYWORDS, pyswitch_remove_doc},
    {"get_qrox",
     (PyCFunction)pySwitch::get_qrox,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_get_qrox_doc},
    {"get_qrox_from_name",
     (PyCFunction)pySwitch::get_qrox_from_name,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_get_qrox_from_name_doc},
    {"get_state", (PyCFunction)pySwitch::get_state, METH_NOARGS, pyswitch_get_state_doc},
    {"load_state",
     (PyCFunction)pySwitch::load_state,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_load_state_doc},
    {"reset_state",
     (PyCFunction)pySwitch::reset_state,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_reset_state_doc},
    {"list_classes",
     (PyCFunction)pySwitch::list_classes,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_list_classes_doc},
    {"classes_doc",
     (PyCFunction)pySwitch::classes_doc,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_classes_doc_doc},
    {"class_doc",
     (PyCFunction)pySwitch::class_doc,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_class_doc_doc},
    {"list_quids",
     (PyCFunction)pySwitch::list_quids,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_list_quids_doc},
    {"quids_descr",
     (PyCFunction)pySwitch::quids_descr,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_quids_descr_doc},
    {"quid_descr",
     (PyCFunction)pySwitch::quid_descr,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_quid_descr_doc},
    {nullptr}};

PyDoc_STRVAR(pyquid_switcher_doc,
             "Switcher objects.\n"
             "Entry point for creating, removing and accessing quiddities.\n");

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
    pyquid_switcher_doc,                                        /* tp_doc */
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
