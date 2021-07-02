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

#include <switcher/infotree/information-tree.hpp>
#include <switcher/logger/console.hpp>
#include <switcher/logger/silent.hpp>

#include "./pyinfotree.hpp"
#include "./pyquiddity.hpp"

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
  PyObject* configFile = nullptr;
  PyObject* showDebug = nullptr;

  static char* kwlist[] = {(char*)"name", (char*)"config", (char*)"debug", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|OO", kwlist, &name, &configFile, &showDebug))
    return -1;

  self->name = name;
  if (showDebug && PyBool_Check(showDebug) && showDebug==Py_True) {
    self->switcher = Switcher::make_switcher<log::Console>(PyUnicode_AsUTF8(self->name));
  } else {
    self->switcher = Switcher::make_switcher<log::Silent>(PyUnicode_AsUTF8(self->name));
  }

  if (!self->switcher) {
    return -1;
  }

  self->sig_reg = std::make_unique<sig_registering_t>();

  if (configFile) {
    if (!self->switcher->conf<MPtr(&Configuration::from_file)>(PyUnicode_AsUTF8(configFile)))
      return -1;
  }

  self->interpreter_state = PyThreadState_Get()->interp;

  return 0;
}

void pySwitch::Switcher_dealloc(pySwitchObject* self) {
  // cleaning signal subscription
  self->switcher->quids<MPtr(&quiddity::Container::reset_create_remove_cb)>();
  for (const auto& it : self->sig_reg->callbacks) {
    Py_XDECREF(it.second);
  }
  for (auto& it : self->sig_reg->user_data) {
    Py_XDECREF(it.second);
  }
  self->sig_reg.reset();

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

PyDoc_STRVAR(
    pyswitch_load_bundles_doc,
    "Load bundles description from a JSON object and make them available for creation."
    "The description must be a stringified JSON object containing a valid bundle description.\n"
    "Arguments: (description)\n"
    "Returns: True or False.\n");

PyObject* pySwitch::load_bundles(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* description = nullptr;
  static char* kwlist[] = {(char*)"description", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &description)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  if (!self->switcher.get()->load_bundle_from_config(description)) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyswitch_create_doc,
             "Create a quiddity. The name and the config are optional."
             "The config (an InfoTree) overrides the switcher configuration file  \n"
             "Arguments: (type, name, config)\n"
             "Returns: the created quiddity object (pyquid.Quiddity), or None\n");

PyObject* pySwitch::create(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* type = nullptr;
  const char* name = nullptr;
  PyObject* pyinfotree = nullptr;
  static char* kwlist[] = {(char*)"type", (char*)"name", (char*)"config", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|sO", kwlist, &type, &name, &pyinfotree)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (pyinfotree &&
      !PyObject_IsInstance(pyinfotree, reinterpret_cast<PyObject*>(&pyInfoTree::pyType))) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  PyObject* argList = nullptr;
  if (!name && !pyinfotree)
    argList = Py_BuildValue("(Os)", (PyObject*)self, type);
  else if (name && !pyinfotree)
    argList = Py_BuildValue("(Oss)", (PyObject*)self, type, name);
  else if (!name && pyinfotree)
    argList = Py_BuildValue("(Oss)", (PyObject*)self, type, "", pyinfotree);
  else if (name && pyinfotree)
    argList = Py_BuildValue("(OssO)", (PyObject*)self, type, name, pyinfotree);

  PyObject* obj = PyObject_CallObject((PyObject*)&pyQuiddity::pyType, argList);
  Py_XDECREF(argList);
  return obj;
}

PyDoc_STRVAR(pyswitch_remove_doc,
             "Remove a quiddity.\n"
             "Arguments: (id)\n"
             "Returns: true or false\n");

PyObject* pySwitch::remove(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  long int id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "l", kwlist, &id)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  if (!self->switcher->quids<MPtr(&quiddity::Container::remove)>(id)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyswitch_get_quid_doc,
             "Get a Quiddty object from its id.\n"
             "Arguments: (id)\n"
             "Returns: a Quiddity object (pyquid.Quiddity), or None.\n");

PyObject* pySwitch::get_quid(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  long int id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "l", kwlist, &id)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto qrox = self->switcher->quids<MPtr(&quiddity::Container::get_qrox)>(id);
  if (!qrox) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto quid_capsule = PyCapsule_New(static_cast<void*>(qrox.get()), nullptr, nullptr);
  PyObject* argList = Py_BuildValue("(O)", quid_capsule);
  PyObject* obj = PyObject_CallObject((PyObject*)&pyQuiddity::pyType, argList);
  Py_XDECREF(argList);
  Py_XDECREF(quid_capsule);
  return obj;
}

PyDoc_STRVAR(pyswitch_get_quid_id_doc,
             "Get a quiddity id from its nickname.\n"
             "Arguments: nickname\n"
             "Returns: the id (strictly positive long int)\n");

PyObject* pySwitch::get_quid_id(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* nickname = nullptr;
  static char* kwlist[] = {(char*)"nickname", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &nickname)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  return PyLong_FromSize_t(self->switcher->quids<MPtr(&quiddity::Container::get_id)>(nickname));
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
  auto class_list = self->switcher->factory<MPtr(&quiddity::Factory::get_class_list)>();
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
      infotree::json::serialize(
          self->switcher->factory<MPtr(&quiddity::Factory::get_classes_doc)>().get())
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
      infotree::json::serialize(self->switcher->factory<MPtr(&quiddity::Factory::get_classes_doc)>()
                                    ->get_tree(std::string(".classes.") + class_name)
                                    .get())
          .c_str());
}

PyDoc_STRVAR(pyswitch_list_quids_doc,
             "Get the nicknames.\n"
             "Arguments: (None)\n"
             "Returns: List of nicknames.\n");

PyObject* pySwitch::list_quids(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto quids = self->switcher->quids<MPtr(&quiddity::Container::get_nicknames)>();
  PyObject* result = PyList_New(quids.size());
  for (unsigned int i = 0; i < quids.size(); ++i) {
    PyList_SetItem(result, i, Py_BuildValue("s", quids[i].c_str()));
  }
  return result;
}

PyDoc_STRVAR(pyswitch_list_ids_doc,
             "Get the quiddity ids.\n"
             "Arguments: (None)\n"
             "Returns: List of ids.\n");

PyObject* pySwitch::list_ids(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto quids = self->switcher->quids<MPtr(&quiddity::Container::get_ids)>();
  PyObject* result = PyList_New(quids.size());
  for (unsigned int i = 0; i < quids.size(); ++i) {
    // "n" for Py_ssize_t (size_t)
    PyList_SetItem(result, i, Py_BuildValue("n", quids[i]));
  }
  return result;
}

PyDoc_STRVAR(pyswitch_quids_descr_doc,
             "Get a JSON description of all the instanciated quiddities.\n"
             "Arguments: (None)\n"
             "Returns: the JSON-formated description.\n");

PyObject* pySwitch::quids_descr(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  return PyUnicode_FromString(
      infotree::json::serialize(
          self->switcher->quids<MPtr(&quiddity::Container::get_quiddities_description)>().get())
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
      infotree::json::serialize(
          self->switcher->quids<MPtr(&quiddity::Container::get_quiddity_description)>(id).get())
          .c_str());
}

bool pySwitch::subscribe_to_signal(pySwitchObject* self,
                                   const std::string signal_name,
                                   PyObject* cb,
                                   PyObject* user_data) {
  auto signalCb = [cb, user_data, self](quiddity::qid_t id) {
    bool has_gil = (1 == PyGILState_Check()) ? true : false;
    PyThreadState* m_state = nullptr;
    if (!has_gil) {
      m_state = PyThreadState_New(self->interpreter_state);
      PyEval_RestoreThread(m_state);
    }
    PyObject* arglist;
    if (user_data)
      arglist = Py_BuildValue("(nO)", id, user_data);
    else
      arglist = Py_BuildValue("(n)", id);
    PyObject* pyobjresult = PyEval_CallObject(cb, arglist);
    PyObject* pyerr = PyErr_Occurred();
    if (pyerr != nullptr) PyErr_Print();
    Py_DECREF(arglist);
    Py_XDECREF(pyobjresult);
    if (!has_gil) {
      PyEval_SaveThread();
      if (m_state) {
        PyThreadState_Clear(m_state);
        PyThreadState_Delete(m_state);
      }
    }
  };

  unsigned int reg_id = 0;
  if ("on-quiddity-created" == signal_name) {
    reg_id = self->switcher->quids<MPtr(&quiddity::Container::register_creation_cb)>(signalCb);
  } else if ("on-quiddity-removed" == signal_name) {
    reg_id = self->switcher->quids<MPtr(&quiddity::Container::register_removal_cb)>(signalCb);
  } else {
    return false;
  }

  if (0 == reg_id) return false;
  Py_INCREF(cb);
  self->sig_reg->callbacks.emplace(signal_name, cb);
  self->sig_reg->signals.emplace(signal_name, reg_id);
  if (user_data) {
    Py_INCREF(user_data);
    self->sig_reg->user_data.emplace(signal_name, user_data);
  }
  return true;
}

PyDoc_STRVAR(pyswitch_subscribe_doc,
             "Subscribe to a signal. The callback has two argument(s): 'value'"
             "(JSON representation of the signal's value) and 'user_data', "
             "if 'subscribe' has been invoked with 'user_data'.\n"
             "Arguments: (name, callback, user_data) where 'name' is either 'on-quiddity-created' "
             "or 'on-quiddity-removed'."
             "Note that 'user_data' is optional\n"
             "Returns: True or False\n");

PyObject* pySwitch::subscribe(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* name = nullptr;
  PyObject* cb = nullptr;
  PyObject* user_data = nullptr;

  static char* kwlist[] = {(char*)"name", (char*)"cb", (char*)"user_data", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|O", kwlist, &name, &cb, &user_data)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (!PyCallable_Check(cb)) {
    PyErr_SetString(PyExc_TypeError, "pySwitch callback argument must be callable");
    return static_cast<PyObject*>(nullptr);
  }

  if (subscribe_to_signal(self, name, cb, user_data)) {
    Py_INCREF(Py_True);
    return Py_True;
  }

  // no subscription worked
  Py_INCREF(Py_False);
  return Py_False;
}

bool pySwitch::unsubscribe_from_signal(pySwitchObject* self, const std::string signal_name) {
  auto found = self->sig_reg->signals.find(signal_name);
  if (self->sig_reg->signals.end() == found) return false;
  if (found->first == "on-quiddity-created") {
    self->switcher->quids<MPtr(&quiddity::Container::unregister_creation_cb)>(found->second);
  } else {
    self->switcher->quids<MPtr(&quiddity::Container::unregister_removal_cb)>(found->second);
  }
  auto cb = self->sig_reg->callbacks.find(signal_name);
  Py_XDECREF(cb->second);
  self->sig_reg->callbacks.erase(cb);
  auto user_data = self->sig_reg->user_data.find(signal_name);
  if (self->sig_reg->user_data.end() != user_data) {
    Py_XDECREF(user_data->second);
    self->sig_reg->user_data.erase(user_data);
  }
  self->sig_reg->signals.erase(signal_name);
  return true;
}

PyDoc_STRVAR(
    pyswitch_unsubscribe_doc,
    "Unsubscribe from a signal.\n"
    "Arguments: (name) where 'name' is either 'on-quiddity-created' or 'on-quiddity-removed'.\n"
    "Returns: True or False\n");

PyObject* pySwitch::unsubscribe(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* name = nullptr;
  static char* kwlist[] = {(char*)"name", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &name)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  if (unsubscribe_from_signal(self, name)) {
    Py_INCREF(Py_True);
    return Py_True;
  }

  // no unsubscribe worked
  Py_INCREF(Py_False);
  return Py_False;
}

PyMethodDef pySwitch::pySwitch_methods[] = {
    {"name", (PyCFunction)pySwitch::name, METH_NOARGS, pyswitch_name_doc},
    {"version", (PyCFunction)pySwitch::version, METH_NOARGS, pyswitch_version_doc},
    {"load_bundles",
     (PyCFunction)pySwitch::load_bundles,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_load_bundles_doc},
    {"create", (PyCFunction)pySwitch::create, METH_VARARGS | METH_KEYWORDS, pyswitch_create_doc},
    {"remove", (PyCFunction)pySwitch::remove, METH_VARARGS | METH_KEYWORDS, pyswitch_remove_doc},
    {"get_quid",
     (PyCFunction)pySwitch::get_quid,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_get_quid_doc},
    {"get_quid_id",
     (PyCFunction)pySwitch::get_quid_id,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_get_quid_id_doc},
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
    {"list_ids",
     (PyCFunction)pySwitch::list_ids,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_list_ids_doc},
    {"quids_descr",
     (PyCFunction)pySwitch::quids_descr,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_quids_descr_doc},
    {"quid_descr",
     (PyCFunction)pySwitch::quid_descr,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_quid_descr_doc},
    {"subscribe",
     (PyCFunction)pySwitch::subscribe,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_subscribe_doc},
    {"unsubscribe",
     (PyCFunction)pySwitch::unsubscribe,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_unsubscribe_doc},
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
