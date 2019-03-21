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
#include <chrono>
#include "./pyinfotree.hpp"
#include "./ungiled.hpp"
#include "switcher/scope-exit.hpp"

PyDoc_STRVAR(pyquiddity_set_doc,
             "Set the value of a property with its name and a string value.\n"
             "Arguments: (name, value)\n"
             "Returns: true of false\n");

PyObject* pyQuiddity::set(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* property = nullptr;
  PyObject* value = nullptr;
  static char* kwlist[] = {(char*)"property", (char*)"value", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO", kwlist, &property, &value)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  PyObject* val_str = nullptr;
  PyObject* repr = nullptr;
  On_scope_exit {
    if (val_str) Py_XDECREF(val_str);
    if (repr) Py_XDECREF(repr);
  };
  if (PyBool_Check(value)) {
    if (!pyquid::ungiled(std::function([&]() {
          return self->quid->prop<MPtr(&PContainer::set_str_str)>(
              property, (value == Py_True) ? "true" : "false");
        }))) {
      Py_INCREF(Py_False);
      return Py_False;
    }
    Py_INCREF(Py_True);
    return Py_True;
  }
  if (!PyObject_TypeCheck(value, &PyUnicode_Type)) {
    repr = PyObject_Repr(value);
    val_str = PyUnicode_AsEncodedString(repr, "utf-8", "Error ");
  } else {
    val_str = PyUnicode_AsEncodedString(value, "utf-8", "Error ");
  }
  if (!pyquid::ungiled(std::function([&]() {
        return self->quid->prop<MPtr(&PContainer::set_str_str)>(property,
                                                                PyBytes_AS_STRING(val_str));
      }))) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyquiddity_get_doc,
             "Get the value of a property from its name.\n"
             "Arguments: (name)\n"
             "Returns: the value (string)\n");

PyObject* pyQuiddity::get(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* property = nullptr;
  static char* kwlist[] = {(char*)"name", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &property)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  auto prop_id = self->quid->prop<MPtr(&PContainer::get_id)>(property);
  if (0 == prop_id) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  return pyInfoTree::any_to_pyobject(pyquid::ungiled(
      std::function([&]() { return self->quid->prop<MPtr(&PContainer::get_any)>(prop_id); })));
}

PyDoc_STRVAR(pyquiddity_invoke_doc,
             "Invoke a method with its names and arguments.\n"
             "Arguments: (method, args=[])\n"
             "Returns: the value returned by the method\n");

PyObject* pyQuiddity::invoke(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* method = nullptr;
  PyObject* inv_args = nullptr;
  static char* kwlist[] = {(char*)"method", (char*)"args", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|O", kwlist, &method, &inv_args)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  Py_ssize_t list_size = 0;
  if (nullptr != inv_args) {
    if (PyList_Check(inv_args)) {  // a list argument is given
      list_size = PyList_Size(inv_args);
    } else {  // something else is given as argument
      Py_INCREF(Py_None);
      return Py_None;
    }
  }
  auto tuple_args = std::string();
  for (auto i = 0; i < list_size; ++i) {
    PyObject* item = nullptr;
    PyObject* item_str = nullptr;
    PyObject* repr = nullptr;
    On_scope_exit {
      if (item_str) Py_XDECREF(item_str);
      if (repr) Py_XDECREF(repr);
    };
    item = PyList_GetItem(inv_args, i);
    if (!PyObject_TypeCheck(item, &PyUnicode_Type)) {
      repr = PyObject_Repr(item);
      item_str = PyUnicode_AsEncodedString(repr, "utf-8", "Error ");
    } else {
      item_str = PyUnicode_AsEncodedString(item, "utf-8", "Error ");
    }

    if (tuple_args.empty())
      tuple_args = serialize::esc_for_tuple(PyBytes_AS_STRING(item_str));
    else
      tuple_args = tuple_args + "," + serialize::esc_for_tuple(PyBytes_AS_STRING(item_str));
  }

  BoolAny res = pyquid::ungiled(std::function([&]() {
    return self->quid->meth<MPtr(&MContainer::invoke_any)>(
        self->quid->meth<MPtr(&MContainer::get_id)>(method), tuple_args);
  }));

  if (!res) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  return pyInfoTree::any_to_pyobject(res.any());
}

PyDoc_STRVAR(pyquiddity_invoke_async_doc,
             "Asynchronous invocation of a method with its names and arguments. "
             "Returned value can be obtained with a call back.\n"
             "Arguments: (method, args=[], on_done_cb=None, user_data=None)\n"
             "Returns: None\n");

PyObject* pyQuiddity::invoke_async(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* method = nullptr;
  PyObject* inv_args = nullptr;
  PyObject* cb = nullptr;
  PyObject* user_data = nullptr;
  static char* kwlist[] = {
      (char*)"method", (char*)"args", (char*)"cb", (char*)"user_data", nullptr};
  if (!PyArg_ParseTupleAndKeywords(
          args, kwds, "s|OOO", kwlist, &method, &inv_args, &cb, &user_data)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  Py_ssize_t list_size = 0;
  if (nullptr != inv_args) {
    if (PyList_Check(inv_args)) {  // a list argument is given
      list_size = PyList_Size(inv_args);
    } else {  // something else is given as argument
      Py_INCREF(Py_None);
      return Py_None;
    }
  }
  auto tuple_args = std::string();
  for (auto i = 0; i < list_size; ++i) {
    PyObject* item = nullptr;
    PyObject* item_str = nullptr;
    PyObject* repr = nullptr;
    On_scope_exit {
      if (item_str) Py_XDECREF(item_str);
      if (repr) Py_XDECREF(repr);
    };
    item = PyList_GetItem(inv_args, i);
    if (!PyObject_TypeCheck(item, &PyUnicode_Type)) {
      repr = PyObject_Repr(item);
      item_str = PyUnicode_AsEncodedString(repr, "utf-8", "Error ");
    } else {
      item_str = PyUnicode_AsEncodedString(item, "utf-8", "Error ");
    }

    if (tuple_args.empty())
      tuple_args = serialize::esc_for_tuple(PyBytes_AS_STRING(item_str));
    else
      tuple_args = tuple_args + "," + serialize::esc_for_tuple(PyBytes_AS_STRING(item_str));
  }

  if (cb != nullptr && !PyCallable_Check(cb)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  Py_INCREF(cb);
  Py_INCREF(user_data);
  self->async_invocations->emplace_back(std::async(
      std::launch::async, [self, cb, user_data, meth = std::string(method), tuple_args]() {
        auto res = self->quid->meth<MPtr(&MContainer::invoke_any)>(
            self->quid->meth<MPtr(&MContainer::get_id)>(meth), tuple_args);
        bool has_gil = (1 == PyGILState_Check()) ? true : false;
        PyThreadState* m_state = nullptr;
        if (!has_gil) {
          m_state = PyThreadState_New(self->interpreter_state);
          PyEval_RestoreThread(m_state);
        }
        PyObject* res_object = pyInfoTree::any_to_pyobject(res.any());
        PyObject* arglist;
        if (user_data)
          arglist = Py_BuildValue("(OO)", res_object, user_data);
        else
          arglist = Py_BuildValue("(O)", res_object);
        PyObject* pyobjresult = PyEval_CallObject(cb, arglist);
        PyObject* pyerr = PyErr_Occurred();
        if (pyerr != nullptr) PyErr_Print();
        Py_DECREF(cb);
        Py_DECREF(user_data);
        Py_DECREF(res_object);
        Py_DECREF(arglist);
        Py_XDECREF(pyobjresult);
        if (!has_gil) {
          PyEval_SaveThread();
          PyThreadState_Clear(m_state);
          PyThreadState_Delete(m_state);
        }
      }));

  // cleaning old invocations
  while (!self->async_invocations->empty() &&
         self->async_invocations->front().wait_for(std::chrono::seconds(0)) ==
             std::future_status::ready)
    self->async_invocations->pop_front();

  Py_INCREF(Py_None);
  return Py_None;
}

PyDoc_STRVAR(pyquiddity_make_shmpath_doc,
             "Get the shmdata path the quiddity will generate internally for the given suffix.\n"
             "Arguments: (suffix)\n"
             "Returns: the shmdata path (string)\n");

PyObject* pyQuiddity::make_shmpath(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* suffix = nullptr;
  static char* kwlist[] = {(char*)"suffix", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &suffix)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  return PyUnicode_FromString(self->quid->make_shmpath(suffix).c_str());
}

PyDoc_STRVAR(pyquiddity_get_user_tree_doc,
             "Get the user data tree attached to the Quiddity. Note this tree is saved with the "
             "Quiddity state and can be retrieved when a switcher save file is loaded.\n"
             "Arguments: none\n"
             "Returns: the user data (InfoTree)\n");

PyObject* pyQuiddity::get_user_tree(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  auto* tree = self->quid->user_data<MPtr(&InfoTree::get_tree)>(".").get();
  return pyInfoTree::make_pyobject_from_c_ptr(tree, false);
}

PyDoc_STRVAR(pyquiddity_get_info_doc,
             "Get a value in the InfoTree.\n"
             "Arguments: (path)\n"
             "Returns: the value\n");

PyObject* pyQuiddity::get_info(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }

  return pyInfoTree::any_to_pyobject(self->quid->tree<MPtr(&InfoTree::branch_get_value)>(path));
}

bool pyQuiddity::subscribe_to_signal(pyQuiddityObject* self,
                                     const char* signal_name,
                                     PyObject* cb,
                                     PyObject* user_data) {
  auto sig_id = self->quid->sig<MPtr(&SContainer::get_id)>(signal_name);
  if (0 == sig_id) return false;
  auto reg_id = self->quid->sig<MPtr(&SContainer::subscribe)>(
      sig_id, [cb, user_data, self](const InfoTree::ptr& tree) {
        bool has_gil = (1 == PyGILState_Check()) ? true : false;
        PyThreadState* m_state;
        if (!has_gil) {
          m_state = PyThreadState_New(self->interpreter_state);
          PyEval_RestoreThread(m_state);
        }
        PyObject* arglist;
        if (user_data)
          arglist = Py_BuildValue("(sO)", (char*)tree->serialize_json(".").c_str(), user_data);
        else
          arglist = Py_BuildValue("(s)", (char*)tree->serialize_json(".").c_str());
        PyObject* pyobjresult = PyEval_CallObject(cb, arglist);
        PyObject* pyerr = PyErr_Occurred();
        if (pyerr != nullptr) PyErr_Print();
        Py_DECREF(arglist);
        Py_XDECREF(pyobjresult);
        if (!has_gil) {
          PyEval_SaveThread();
          PyThreadState_Clear(m_state);
          PyThreadState_Delete(m_state);
        }
      });
  if (0 == reg_id) return false;
  Py_INCREF(cb);
  self->sig_reg->callbacks.emplace(sig_id, cb);
  self->sig_reg->signals.emplace(sig_id, reg_id);
  if (user_data) {
    Py_INCREF(user_data);
    self->sig_reg->user_data.emplace(sig_id, user_data);
  }
  return true;
}

bool pyQuiddity::subscribe_to_property(pyQuiddityObject* self,
                                       const char* prop_name,
                                       PyObject* cb,
                                       PyObject* user_data) {
  auto prop_id = self->quid->prop<MPtr(&PContainer::get_id)>(prop_name);
  if (0 == prop_id) return false;
  auto reg_id =
      self->quid->prop<MPtr(&PContainer::subscribe)>(prop_id, [prop_id, cb, self, user_data]() {
        bool has_gil = (1 == PyGILState_Check()) ? true : false;
        PyThreadState* m_state;
        if (!has_gil) {
          m_state = PyThreadState_New(self->interpreter_state);
          PyEval_RestoreThread(m_state);
        }
        PyObject* arglist;
        if (user_data)
          arglist = Py_BuildValue(
              "(OO)",
              pyInfoTree::any_to_pyobject(self->quid->prop<MPtr(&PContainer::get_any)>(prop_id)),
              user_data);
        else
          arglist = Py_BuildValue(
              "(O)",
              pyInfoTree::any_to_pyobject(self->quid->prop<MPtr(&PContainer::get_any)>(prop_id)));
        PyObject* pyobjresult = PyEval_CallObject(cb, arglist);
        PyObject* pyerr = PyErr_Occurred();
        if (pyerr != nullptr) PyErr_Print();
        Py_DECREF(arglist);
        Py_XDECREF(pyobjresult);
        if (!has_gil) {
          PyEval_SaveThread();
          PyThreadState_Clear(m_state);
          PyThreadState_Delete(m_state);
        }
      });
  if (0 == reg_id) return false;
  Py_INCREF(cb);
  self->prop_reg->callbacks.emplace(prop_id, cb);
  self->prop_reg->props.emplace(prop_id, reg_id);
  if (user_data) {
    Py_INCREF(user_data);
    self->prop_reg->user_data.emplace(prop_id, user_data);
  }
  return true;
}

PyDoc_STRVAR(pyquiddity_subscribe_doc,
             "Subscribe to a signal or to a property. The callback has two argument(s): value (of "
             "the property or the json representation of the value for signals), and the user_data "
             "if subscribe has been invoked with a user_data.\n"
             "Arguments: (name, callback, user_data) where name is a signal name or a property "
             "name. Note that user_data is optional\n"
             "Returns: True or False\n");

PyObject* pyQuiddity::subscribe(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* name = nullptr;
  PyObject* cb = nullptr;
  PyObject* user_data = nullptr;

  static char* kwlist[] = {(char*)"name", (char*)"cb", (char*)"user_data", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO|O", kwlist, &name, &cb, &user_data)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (!PyCallable_Check(cb)) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  if (subscribe_to_property(self, name, cb, user_data)) {
    Py_INCREF(Py_True);
    return Py_True;
  }

  if (subscribe_to_signal(self, name, cb, user_data)) {
    Py_INCREF(Py_True);
    return Py_True;
  }

  // no subscription worked
  Py_INCREF(Py_False);
  return Py_False;
}

bool pyQuiddity::unsubscribe_from_property(pyQuiddityObject* self, const char* prop_name) {
  auto prop_id = self->quid->prop<MPtr(&PContainer::get_id)>(prop_name);
  if (0 == prop_id) return false;
  auto found = self->prop_reg->props.find(prop_id);
  if (self->prop_reg->props.end() == found) return false;
  auto unsubscribed = self->quid->prop<MPtr(&PContainer::unsubscribe)>(prop_id, found->second);
  if (!unsubscribed) return false;
  auto cb = self->prop_reg->callbacks.find(prop_id);
  Py_XDECREF(cb->second);
  self->prop_reg->callbacks.erase(cb);
  auto user_data = self->prop_reg->user_data.find(prop_id);
  if (self->prop_reg->user_data.end() != user_data) {
    Py_XDECREF(user_data->second);
    self->prop_reg->user_data.erase(user_data);
  }
  self->prop_reg->props.erase(prop_id);
  return true;
}

bool pyQuiddity::unsubscribe_from_signal(pyQuiddityObject* self, const char* signal_name) {
  auto sig_id = self->quid->sig<MPtr(&SContainer::get_id)>(signal_name);
  if (0 == sig_id) return false;
  auto found = self->sig_reg->signals.find(sig_id);
  if (self->sig_reg->signals.end() == found) return false;
  auto unsubscribed = self->quid->sig<MPtr(&SContainer::unsubscribe)>(sig_id, found->second);
  if (!unsubscribed) return false;
  auto cb = self->sig_reg->callbacks.find(sig_id);
  Py_XDECREF(cb->second);
  self->sig_reg->callbacks.erase(cb);
  auto user_data = self->sig_reg->user_data.find(sig_id);
  if (self->sig_reg->user_data.end() != user_data) {
    Py_XDECREF(user_data->second);
    self->sig_reg->user_data.erase(user_data);
  }
  self->sig_reg->signals.erase(sig_id);
  return true;
}

PyDoc_STRVAR(pyquiddity_unsubscribe_doc,
             "Unsubscribe from a signal or a property.\n"
             "Arguments: (name)\n"
             "Returns: True or False\n");

PyObject* pyQuiddity::unsubscribe(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
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

  if (unsubscribe_from_signal(self, name)) {
    Py_INCREF(Py_True);
    return Py_True;
  }

  // no unsubscribe worked
  Py_INCREF(Py_False);
  return Py_False;
}

PyDoc_STRVAR(pyquiddity_get_info_tree_as_json_doc,
             "Get json serialization of InfoTree the subtree.\n"
             "Arguments: (path)\n"
             "Returns: the json string\n");

PyObject* pyQuiddity::get_info_tree_as_json(pyQuiddityObject* self,
                                            PyObject* args,
                                            PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (nullptr == path) path = ".";

  std::string res = pyquid::ungiled(
      std::function([&]() { return self->quid->tree<MPtr(&InfoTree::serialize_json)>(path); }));

  return PyUnicode_FromString(res.c_str());
}

PyDoc_STRVAR(pyquiddity_get_signal_id_doc,
             "Get the id of a given signal.\n"
             "Arguments: (name)\n"
             "Returns: the id \n");

PyObject* pyQuiddity::get_signal_id(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* signal = nullptr;
  static char* kwlist[] = {(char*)"name", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &signal)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  return PyLong_FromLong(self->quid->sig<MPtr(&SContainer::get_id)>(signal));
}

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
  self->sig_reg = std::make_unique<sig_registering_t>();
  self->prop_reg = std::make_unique<prop_registering_t>();
  self->async_invocations = std::make_unique<std::list<std::future<void>>>();
  self->interpreter_state = PyThreadState_Get()->interp;
  PyEval_InitThreads();
  return 0;
}

void pyQuiddity::Quiddity_dealloc(pyQuiddityObject* self) {
  // cleaning signal subscription
  for (const auto& it : self->sig_reg->callbacks) {
    auto found = self->sig_reg->signals.find(it.first);
    self->quid->sig<MPtr(&SContainer::unsubscribe)>(found->first, found->second);
    Py_XDECREF(it.second);
  }
  for (auto& it : self->sig_reg->user_data) {
    Py_XDECREF(it.second);
  }
  self->sig_reg.reset();

  // cleaning prop subscription
  for (const auto& it : self->prop_reg->callbacks) {
    auto found = self->prop_reg->props.find(it.first);
    self->quid->prop<MPtr(&PContainer::unsubscribe)>(found->first, found->second);
    Py_XDECREF(it.second);
  }
  for (auto& it : self->prop_reg->user_data) {
    Py_XDECREF(it.second);
  }
  self->prop_reg.reset();
  self->async_invocations.reset();

  // cleaning self
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyMethodDef pyQuiddity::pyQuiddity_methods[] = {
    {"set", (PyCFunction)pyQuiddity::set, METH_VARARGS | METH_KEYWORDS, pyquiddity_set_doc},
    {"get", (PyCFunction)pyQuiddity::get, METH_VARARGS | METH_KEYWORDS, pyquiddity_get_doc},
    {"invoke",
     (PyCFunction)pyQuiddity::invoke,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_invoke_doc},
    {"invoke_async",
     (PyCFunction)pyQuiddity::invoke_async,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_invoke_async_doc},
    {"make_shmpath",
     (PyCFunction)pyQuiddity::make_shmpath,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_make_shmpath_doc},
    {"get_user_tree",
     (PyCFunction)pyQuiddity::get_user_tree,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_get_user_tree_doc},
    {"get_info",
     (PyCFunction)pyQuiddity::get_info,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_get_info_doc},
    {"get_info_tree_as_json",
     (PyCFunction)pyQuiddity::get_info_tree_as_json,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_get_info_tree_as_json_doc},
    {"subscribe",
     (PyCFunction)pyQuiddity::subscribe,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_subscribe_doc},
    {"unsubscribe",
     (PyCFunction)pyQuiddity::unsubscribe,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_unsubscribe_doc},
    {"get_signal_id",
     (PyCFunction)pyQuiddity::get_signal_id,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_get_signal_id_doc},
    {nullptr}};

PyDoc_STRVAR(pyquid_quiddity_doc,
             "Quiddity objects.\n"
             "Quiddities are services that can be instantiated by a switcher. "
             "Communication with a Quiddity is achieved through:\n"
             "   - property set, get and subscribe\n"
             "   - method invocation\n"
             "   - InfoTree request\n"
             "   - specific configuration");

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
    pyquid_quiddity_doc,                                        /* tp_doc */
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
