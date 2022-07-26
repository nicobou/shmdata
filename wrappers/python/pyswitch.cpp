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

#include <sstream>
#include <string>
#include <switcher/infotree/information-tree.hpp>

#include "./pyinfotree.hpp"
#include "./pyquiddity.hpp"

PyObject* InterpType(const char* type_name, const char* module_name) {
  PyObject *key = PyUnicode_FromString(type_name), *globals = PyEval_GetGlobals();
  auto path = fs::path(PYQUID_SCRIPTS_DIR) / module_name;
  if (!fs::exists(path)) {
    // use source files instead of installed files
    path = fs::path(PYQUID_SCRIPTS_DIR_LOCAL) / module_name;
  }
  if (!PyDict_Contains(globals, key)) {
    PyObject *func = PyDict_GetItemString(PyEval_GetBuiltins(), "open"),
             *meth = PyUnicode_FromString("read"), *fname = PyUnicode_FromString(path.c_str()),
             *file = PyObject_CallFunctionObjArgs(func, fname, nullptr);
    // read content from opened file
    PyObject* content = PyObject_CallMethodObjArgs(file, meth, nullptr);
    // execute file content
    PyObject* exec = PyDict_GetItemString(PyEval_GetBuiltins(), "exec");
    PyObject_CallFunctionObjArgs(exec, content, globals, nullptr);
    // close file
    Py_XDECREF(meth);
    meth = PyUnicode_FromString("close");
    PyObject_CallMethodObjArgs(file, meth, nullptr);
    for (auto& o : {meth, fname, file, content}) Py_XDECREF(o);
  }
  PyObject* res = PyDict_GetItem(globals, key);
  Py_DECREF(key);
  return res;
}

PyObject* pySwitch::Switcher_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pySwitchObject* self;

  self = (pySwitchObject*)type->tp_alloc(type, 0);
  if (self != nullptr) {
    self->name = PyUnicode_FromString("default");
    self->quiddities = PyList_New(0);
    if (self->name == nullptr || self->quiddities == nullptr) {
      Py_XDECREF(self);
      return nullptr;
    }
  }
  return (PyObject*)self;
}

int pySwitch::Switcher_init(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  PyObject* name = nullptr;
  PyObject* configFile = nullptr;
  int debug = 0;

  static char* kwlist[] = {(char*)"name", (char*)"config", (char*)"debug", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|Op", kwlist, &name, &configFile, &debug))
    return -1;

  self->name = name;
  std::string name_str = PyUnicode_AsUTF8(self->name);

  // init switcher
  self->switcher = Switcher::make_switcher(name_str, debug);

  if (!self->switcher) {
    PyErr_SetString(PyExc_RuntimeError, "Switcher instance could not be created");
    return -1;
  }

  self->sig_reg = std::make_unique<sig_registering_t>();

  if (configFile) {
    if (!self->switcher->conf<MPtr(&Configuration::from_file)>(PyUnicode_AsUTF8(configFile))) {
      PyErr_SetString(PyExc_RuntimeError, "Switcher could not load configuration file");
      return -1;
    }
  }

  // initialize logger
  self->logger =
      PyObject_CallFunctionObjArgs(reinterpret_cast<PyObject*>(&pyLogger::pyType), nullptr);

  // Initialize session by calling the class object with a pyswitch instance as a unique argument
  self->session =
      PyObject_CallFunctionObjArgs(reinterpret_cast<PyObject*>(&pySession::pyType), self, nullptr);
  // @NOTE: This is pretty much like doing the following in Python:
  //
  // class Session:
  //   def __init__(self, instance):
  //     self.instance = instance;
  //
  // class Switcher:
  //   def __init__(self):
  //     self.session = Session(self);
  //
  // Anytime a Switcher instance is initialized, another Session instance is also created.
  // For convenience, both instances might keep a reference to each other.

  // init bundles
  self->bundles = PyObject_CallFunctionObjArgs(reinterpret_cast<PyObject*>(&BundleManager::pyType),
                                               reinterpret_cast<PyObject*>(self),
                                               nullptr);

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

  // decrement refcounts
  for (auto& o : {self->name, self->quiddities, self->session}) Py_XDECREF(o);

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
  return PyUnicode_FromString(self->switcher->name.c_str());
}

PyDoc_STRVAR(pyswitch_version_doc,
             "Get switcher version.\n"
             "Arguments: None\n"
             "Returns: the libswitcher version (string)\n");

PyObject* pySwitch::version(pySwitchObject* self) {
  return PyUnicode_FromString(self->switcher.get()->get_switcher_version().c_str());
}

PyDoc_STRVAR(pyswitch_load_bundles_doc,
             "Load bundles description from a dictionnary and make them available for creation."
             "The description must be a dictionary containing a valid bundle description.\n"
             "A description must contain a main `bundle` key that contains a dictionary of all new bundles. These bundles should be described by a unique name, a `gstreamer` pipeline and a custom documentation. This description is documented in the document `writing-bundles.md`."
             "Arguments: (description)\n"
             "Returns: True or False.\n");

PyObject* pySwitch::load_bundles(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  PyObject* description = nullptr;
  static char* kwlist[] = {(char*)"description", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O", kwlist, &description)) return nullptr;

  PyObject *obj = PyImport_ImportModule("json"), *meth = PyUnicode_FromString("dumps");

  PyObject* res = PyObject_CallMethodObjArgs(obj, meth, description, nullptr);

  const char* config = PyUnicode_AsUTF8(res);

  // decrement refcount
  for (auto& o : {obj, meth, res}) Py_XDECREF(o);

  if (!self->switcher->load_bundle_from_config(config)) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyswitch_create_doc,
             "Create a quiddity. The name and the config are optional."
             "The config (an InfoTree) overrides the switcher configuration file  \n"
             "Arguments: (kind, nickname, config)\n"
             "Returns: the created quiddity object (pyquid.Quiddity), or None\n");

PyObject* pySwitch::create(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* kind = nullptr;
  const char* nickname = nullptr;
  PyObject* pyinfotree = nullptr;
  static char* kwlist[] = {(char*)"kind", (char*)"nickname", (char*)"config", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|sO", kwlist, &kind, &nickname, &pyinfotree)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (pyinfotree &&
      !PyObject_IsInstance(pyinfotree, reinterpret_cast<PyObject*>(&pyInfoTree::pyType))) {
    PyErr_SetString(PyExc_TypeError,
                    "error config argument is not an instance of a pyquid.InfoTree");
    return nullptr;
  }

  PyObject* argList = nullptr;
  if (!nickname && !pyinfotree)
    argList = Py_BuildValue("(Os)", (PyObject*)self, kind);
  else if (nickname && !pyinfotree)
    argList = Py_BuildValue("(Oss)", (PyObject*)self, kind, nickname);
  else if (!nickname && pyinfotree)
    argList = Py_BuildValue("(Oss)", (PyObject*)self, kind, "", pyinfotree);
  else if (nickname && pyinfotree)
    argList = Py_BuildValue("(OssO)", (PyObject*)self, kind, nickname, pyinfotree);

  PyObject* obj = PyObject_CallObject((PyObject*)&pyQuiddity::pyType, argList);
  Py_XDECREF(argList);

  return obj;
}

PyDoc_STRVAR(pyswitch_remove_doc,
             "Remove a quiddity.\n"
             "Arguments: (id)\n"
             "Returns: true or false\n");

PyObject* pySwitch::remove(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  qid_t id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "l", kwlist, &id)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }

  for (Py_ssize_t i = 0; i < PyList_Size(self->quiddities); ++i) {
    auto quid = reinterpret_cast<pyQuiddity::pyQuiddityObject*>(PyList_GetItem(self->quiddities, i))
                    ->quid.lock();
    if (quid && quid->get_id() == id) {
      PyList_SetSlice(self->quiddities, i, i + 1, nullptr);
      break;
    }
  }

  if (!self->switcher->quids<MPtr(&quiddity::Container::remove)>(id)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyswitch_get_quid_doc,
             "Retrieves an existing quiddity object by its identifier \n"
             "and raises a KeyError exception if not found.\n"
             "Arguments: (id)\n"
             "Returns: a Quiddity object (pyquid.Quiddity), or None.\n");

PyObject* pySwitch::get_quid(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  long unsigned int id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "l", kwlist, &id)) return nullptr;

  auto size = PyList_Size(self->quiddities);
  for (Py_ssize_t i = 0; i < size; ++i) {
    PyObject* item = PyList_GetItem(self->quiddities, i);
    if (item && PyObject_TypeCheck(item, &pyQuiddity::pyType)) {
      auto quid = reinterpret_cast<pyQuiddity::pyQuiddityObject*>(item)->quid.lock();
      if (quid->get_id() == id) {
        Py_INCREF(item);
        return item;
      }
    }
  }

  std::ostringstream oss;
  oss << "Quiddity identified by `" << std::to_string(id) << "` does not exist";
  PyErr_SetString(PyExc_KeyError, oss.str().c_str());
  return nullptr;
}

PyDoc_STRVAR(pyswitch_get_quid_id_doc,
             "Get a quiddity id from its nickname.\n"
             "Arguments: nickname\n"
             "Returns: the id (strictly positive long int)\n");

PyObject* pySwitch::get_quid_id(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* nickname = nullptr;
  static char* kwlist[] = {(char*)"nickname", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &nickname)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }

  // also reset references of quiddities
  for (Py_ssize_t i = PyList_Size(self->quiddities) - 1; i >= 0; --i) {
    PyList_SetSlice(self->quiddities, i, i + 1, nullptr);
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (!PyObject_IsInstance(pyinfotree, reinterpret_cast<PyObject*>(&pyInfoTree::pyType))) {
    PyErr_SetString(PyExc_TypeError,
                    "error: state argument is not an instance of a pyquid.InfoTree");
    return nullptr;
  }
  if (!self->switcher->load_state(
          reinterpret_cast<pyInfoTree::pyInfoTreeObject*>(pyinfotree)->tree)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyswitch_list_kinds_doc,
             "Get the list of available kind of quiddity.\n"
             "Arguments: (None)\n"
             "Returns: A list a of string with the kind.\n");

PyObject* pySwitch::list_kinds(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto kind_list = self->switcher->factory<MPtr(&quiddity::Factory::get_kinds)>();
  PyObject* result = PyList_New(kind_list.size());
  for (unsigned int i = 0; i < kind_list.size(); ++i) {
    PyList_SetItem(result, i, Py_BuildValue("s", kind_list[i].c_str()));
  }
  return result;
}

PyDoc_STRVAR(pyswitch_kinds_doc_doc,
             "Get a documentation of all kinds.\n"
             "Arguments: (None)\n"
             "Returns: The documentation of all kinds available.\n");

PyObject* pySwitch::kinds_doc(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto str = infotree::json::serialize(
      self->switcher->factory<MPtr(&quiddity::Factory::get_kinds_doc)>().get());
  // call options
  PyObject *obj = PyImport_ImportModule("json"), *method = PyUnicode_FromString("loads"),
           *arg = PyUnicode_FromString(str.c_str());
  // call function
  PyObject* res = PyObject_CallMethodObjArgs(obj, method, arg, nullptr);
  // decrement refcounts
  for (auto& o : {obj, method, arg}) Py_XDECREF(o);
  // return serialized object
  return res;
}

PyDoc_STRVAR(pyswitch_kind_doc_doc,
             "Get a documentation of a given kinds.\n"
             "Arguments: (kind)\n"
             "Returns: The documentation of the kind.\n");

PyObject* pySwitch::kind_doc(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* kind_name = nullptr;
  static char* kwlist[] = {(char*)"kind", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &kind_name)) return nullptr;
  // get doc as json string
  auto str =
      infotree::json::serialize(self->switcher->factory<MPtr(&quiddity::Factory::get_kinds_doc)>()
                                    ->get_tree(std::string(".kinds.") + kind_name)
                                    .get());
  // call options
  PyObject *obj = PyImport_ImportModule("json"), *method = PyUnicode_FromString("loads"),
           *arg = PyUnicode_FromString(str.c_str());
  // call function
  PyObject* res = PyObject_CallMethodObjArgs(obj, method, arg, nullptr);
  // decrement refcounts
  for (auto& o : {obj, method, arg}) Py_XDECREF(o);
  // return serialized object
  return res;
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
  // serialize infotree to json string
  auto desc = infotree::json::serialize(
      self->switcher->quids<MPtr(&quiddity::Container::get_quiddities_description)>().get());

  if (!desc.c_str()) return nullptr;

  // call options
  PyObject *obj = PyImport_ImportModule("json"), *method = PyUnicode_FromString("loads"),
           *arg = PyUnicode_FromString(desc.c_str());
  // call function
  PyObject* res = PyObject_CallMethodObjArgs(obj, method, arg, nullptr);
  // decrement refcounts
  for (auto& o : {obj, method, arg}) Py_XDECREF(o);
  // return serialized object
  return res;
}

PyDoc_STRVAR(pyswitch_quid_descr_doc,
             "Get a JSON description of a given quiddity.\n"
             "Arguments: (id)\n"
             "Returns: JSON-formated description of the quiddity.\n");

PyObject* pySwitch::quid_descr(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  int id = 0;
  static char* kwlist[] = {(char*)"id", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "i", kwlist, &id) && 0 != id) return nullptr;
  auto desc = infotree::json::serialize(
      self->switcher->quids<MPtr(&quiddity::Container::get_quiddity_description)>(id).get());
  // call options
  PyObject *obj = PyImport_ImportModule("json"), *method = PyUnicode_FromString("loads"),
           *arg = PyUnicode_FromString(desc.c_str());
  // call function
  PyObject* res = PyObject_CallMethodObjArgs(obj, method, arg, nullptr);
  // decrement refcounts
  for (auto& o : {obj, method, arg}) Py_XDECREF(o);
  // return serialized object
  return res;
}

bool pySwitch::subscribe_to_signal(pySwitchObject* self,
                                   const std::string signal_name,
                                   PyObject* cb,
                                   PyObject* user_data) {
  auto signalCb = [cb, user_data, self](quiddity::qid_t id) {
    bool has_gil = (1 == PyGILState_Check()) ? true : false;

    // save the thread state in a local variable
    PyThreadState* m_state = nullptr;

    if (!has_gil) {
      // create a new thread state for the the interpreter interp
      m_state = PyThreadState_New(self->interpreter_state);
      // Acquire the GIL and set the thread state to tstate, which must not be NULL
      PyEval_RestoreThread(m_state);
    }

    // call the python callback
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
      // release the global interpreter lock and reset the thread state to NULL
      PyEval_SaveThread();
      if (m_state) {
        // Reset all information in a thread state object
        // The global interpreter lock must be held
        PyThreadState_Clear(m_state);
        // Destroy a thread state object
        // The global interpreter lock need not be held
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
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|O", kwlist, &name, &cb, &user_data))
    return nullptr;

  if (!PyCallable_Check(cb)) {
    PyErr_SetString(PyExc_TypeError, "argument `callback` is not callable");
    return nullptr;
  }

  if (subscribe_to_signal(self, name, cb, user_data)) Py_RETURN_TRUE;

  // no subscription worked
  Py_RETURN_FALSE;
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }

  if (unsubscribe_from_signal(self, name)) {
    Py_INCREF(Py_True);
    return Py_True;
  }

  // no unsubscribe worked
  Py_INCREF(Py_False);
  return Py_False;
}

PyDoc_STRVAR(pyswitch_list_extra_configs_doc,
             "Get the extra config paths.\n"
             "Arguments: (None)\n"
             "Returns: List of paths.\n");

PyObject* pySwitch::list_extra_configs(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  auto config_paths = self->switcher->conf<MPtr(&Configuration::list_extra_configs)>();
  PyObject* result = PyList_New(config_paths.size());

  for (unsigned int i = 0; i < config_paths.size(); ++i) {
    PyList_SetItem(result, i, Py_BuildValue("s", config_paths[i].c_str()));
  }

  return result;
}

PyDoc_STRVAR(pyswitch_read_extra_config_doc,
             "Read the extra config.\n"
             "Arguments: (name) the name of the extra config file\n"
             "Returns: The extra config file.\n");

PyObject* pySwitch::read_extra_config(pySwitchObject* self, PyObject* args, PyObject* kwds) {
  const char* name = nullptr;
  static char* kwlist[] = {(char*)"name", nullptr};

  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &name)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }

  auto extra_config = self->switcher->conf<MPtr(&Configuration::get_extra_config)>(name);

  if (!extra_config.c_str()) {
    return nullptr;
  } else {
    return PyUnicode_FromString(extra_config.c_str());
  }
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
    {"list_kinds",
     (PyCFunction)pySwitch::list_kinds,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_list_kinds_doc},
    {"kinds_doc",
     (PyCFunction)pySwitch::kinds_doc,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_kinds_doc_doc},
    {"kind_doc",
     (PyCFunction)pySwitch::kind_doc,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_kind_doc_doc},
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
    {"list_extra_configs",
     (PyCFunction)pySwitch::list_extra_configs,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_list_extra_configs_doc},
    {"read_extra_config",
     (PyCFunction)pySwitch::read_extra_config,
     METH_VARARGS | METH_KEYWORDS,
     pyswitch_read_extra_config_doc},
    {nullptr}  // Sentinel
};

PyDoc_STRVAR(pyquid_switcher_doc,
             "The Switcher class.\n"
             "When called, it accepts a `name`, a `config` filepath and a `debug` optional keyword "
             "arguments.\n"
             "It returns a new instance of Switcher which has a `quiddities` attribute listing the "
             "initialized\n"
             "quiddities and a `session` attribute allowing session management.\n");

PyMemberDef pySwitch::pySwitch_members[] = {
    {(char*)"quiddities", T_OBJECT_EX, offsetof(pySwitch::pySwitchObject, quiddities), READONLY},
    {(char*)"session", T_OBJECT_EX, offsetof(pySwitch::pySwitchObject, session), READONLY},
    {(char*)"logger", T_OBJECT_EX, offsetof(pySwitch::pySwitchObject, logger), READONLY},
    {(char*)"bundles", T_OBJECT_EX, offsetof(pySwitch::pySwitchObject, bundles), READONLY},
    {nullptr}};

PyTypeObject pySwitch::pyType = {PyVarObject_HEAD_INIT(nullptr, 0).tp_name = "pyquid.Switcher",
                                 .tp_basicsize = sizeof(pySwitchObject),
                                 .tp_dealloc = (destructor)Switcher_dealloc,
                                 .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
                                 .tp_doc = pyquid_switcher_doc,
                                 .tp_methods = pySwitch_methods,
                                 .tp_members = pySwitch_members,
                                 .tp_init = (initproc)Switcher_init,
                                 .tp_new = (newfunc)Switcher_new};
