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

#include "./pyfclaw.hpp"
#include "./pyinfotree.hpp"
#include "./pyswitch.hpp"
#include "./pywclaw.hpp"
#include "./ungiled.hpp"
#include "switcher/quiddity/claw/claw.hpp"
#include "switcher/utils/scope-exit.hpp"

PyDoc_STRVAR(pyquiddity_set_doc,
             "Set the value of a property with its name and a string value.\n"
             "Arguments: (name, value)\n"
             "Returns: true of false\n");

PyObject* pyQuiddity::set(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* property = nullptr;
  PyObject* value = nullptr;
  static char* kwlist[] = {(char*)"property", (char*)"value", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO", kwlist, &property, &value)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  PyObject* val_str = nullptr;
  PyObject* repr = nullptr;
  On_scope_exit {
    if (val_str) Py_XDECREF(val_str);
    if (repr) Py_XDECREF(repr);
  };
  if (PyBool_Check(value)) {
    if (!pyquid::ungiled(std::function([&]() {
          return self->quid->prop<MPtr(&property::PBag::set_str_str)>(
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
        return self->quid->prop<MPtr(&property::PBag::set_str_str)>(property,
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
  auto prop_id = self->quid->prop<MPtr(&property::PBag::get_id)>(property);
  if (0 == prop_id) {
    PyErr_SetString(PyExc_ValueError, "property not found");
    return static_cast<PyObject*>(nullptr);
  }
  return pyInfoTree::any_to_pyobject(pyquid::ungiled(
      std::function([&]() { return self->quid->prop<MPtr(&property::PBag::get_any)>(prop_id); })));
}

PyDoc_STRVAR(pyquiddity_invoke_doc,
             "Invoke a method with its names and arguments.\n"
             "Arguments: (method, args=[])\n"
             "Returns: the value returned by the method\n"
             "Exception: RuntimeException\n");

PyObject* pyQuiddity::invoke(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* method = nullptr;
  PyObject* inv_args = nullptr;
  static char* kwlist[] = {(char*)"method", (char*)"args", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|O", kwlist, &method, &inv_args)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  Py_ssize_t list_size = 0;
  if (nullptr != inv_args) {
    if (PyList_Check(inv_args)) {  // a list argument is given
      list_size = PyList_Size(inv_args);
    } else {  // something else is given as argument
      PyErr_SetString(PyExc_TypeError,
                      "pyQuiddity invoke method expects a List containing arguments");
      return nullptr;
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
    return self->quid->meth<MPtr(&method::MBag::invoke_any)>(
        self->quid->meth<MPtr(&method::MBag::get_id)>(method), tuple_args);
  }));

  if (!res) {
    PyErr_Format(PyExc_RuntimeError, "%s", res.msg().c_str());
    return nullptr;
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  Py_ssize_t list_size = 0;
  if (nullptr != inv_args) {
    if (PyList_Check(inv_args)) {  // a list argument is given
      list_size = PyList_Size(inv_args);
    } else {  // something else is given as argument
      PyErr_SetString(PyExc_TypeError,
                      "pyQuiddity invoke_async method expects a List containing arguments");
      return nullptr;
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
    PyErr_SetString(
        PyExc_TypeError,
        "pyQuiddity invoke_async method expects a callable object for the on_done_cb argument");
    return nullptr;
  }

  Py_INCREF(cb);
  Py_INCREF(user_data);
  self->async_invocations->emplace_back(std::async(
      std::launch::async, [self, cb, user_data, meth = std::string(method), tuple_args]() {
        auto res = self->quid->meth<MPtr(&method::MBag::invoke_any)>(
            self->quid->meth<MPtr(&method::MBag::get_id)>(meth), tuple_args);
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
        if (pyerr != nullptr) {
          PyErr_Print();
        }
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

PyDoc_STRVAR(pyquiddity_get_user_tree_doc,
             "Get the user data tree attached to the Quiddity. Note this tree is saved with the "
             "Quiddity state and can be retrieved when a switcher save file is loaded.\n"
             "Arguments: none\n"
             "Returns: the user data (InfoTree)\n");

PyObject* pyQuiddity::get_user_tree(pyQuiddityObject* self, PyObject*, PyObject*) {
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  return pyInfoTree::any_to_pyobject(self->quid->tree<MPtr(&InfoTree::branch_get_value)>(path));
}

PyDoc_STRVAR(pyquiddity_get_kind_doc,
             "Get the quiddity kind (a string).\n"
             "Arguments: none\n"
             "Returns: the value\n");

PyObject* pyQuiddity::get_kind(pyQuiddityObject* self, PyObject*, PyObject*) {
  return PyUnicode_FromString(self->quid->get_kind().c_str());
}

PyDoc_STRVAR(pyquiddity_set_nickname_doc,
             "Set the quiddity nickname.\n"
             "Arguments: (nickname)\n"
             "Returns: True or False\n");

PyObject* pyQuiddity::set_nickname(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* nickname = nullptr;
  static char* kwlist[] = {(char*)"nickname", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &nickname)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }

  if (!self->quid->set_nickname(nickname)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyquiddity_nickname_doc,
             "Get the quiddity nickname.\n"
             "Arguments: none\n"
             "Returns: the value\n");

PyObject* pyQuiddity::nickname(pyQuiddityObject* self, PyObject*, PyObject*) {
  return PyUnicode_FromString(self->quid->get_nickname().c_str());
}

PyDoc_STRVAR(pyquiddity_id_doc,
             "Get the quiddity id.\n"
             "Arguments: none\n"
             "Returns: the id (strictly positive long int)\n");

PyObject* pyQuiddity::id(pyQuiddityObject* self, PyObject*, PyObject*) {
  return PyLong_FromSize_t(self->quid->get_id());
}

bool pyQuiddity::subscribe_to_signal(pyQuiddityObject* self,
                                     const char* signal_name,
                                     PyObject* cb,
                                     PyObject* user_data) {
  auto sig_id = self->quid->sig<MPtr(&signal::SBag::get_id)>(signal_name);
  if (0 == sig_id) return false;
  auto reg_id = self->quid->sig<MPtr(&signal::SBag::subscribe)>(
      sig_id, [cb, user_data, self](const InfoTree::ptr& tree) {
        bool has_gil = (1 == PyGILState_Check()) ? true : false;
        PyThreadState* m_state = nullptr;
        if (!has_gil) {
          m_state = PyThreadState_New(self->interpreter_state);
          PyEval_RestoreThread(m_state);
        }
        PyObject* arglist;
        auto tmp_tree = tree->get_copy();
        if (user_data)
          arglist = Py_BuildValue(
              "(OO)", pyInfoTree::make_pyobject_from_c_ptr(tmp_tree.get(), false), user_data);
        else
          arglist =
              Py_BuildValue("(O)", pyInfoTree::make_pyobject_from_c_ptr(tmp_tree.get(), false));
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
  auto prop_id = self->quid->prop<MPtr(&property::PBag::get_id)>(prop_name);
  if (0 == prop_id) return false;
  auto reg_id =
      self->quid->prop<MPtr(&property::PBag::subscribe)>(prop_id, [prop_id, cb, self, user_data]() {
        bool has_gil = (1 == PyGILState_Check()) ? true : false;
        PyThreadState* m_state;
        if (!has_gil) {
          m_state = PyThreadState_New(self->interpreter_state);
          PyEval_RestoreThread(m_state);
        }
        PyObject* arglist;
        if (user_data)
          arglist = Py_BuildValue("(OO)",
                                  pyInfoTree::any_to_pyobject(
                                      self->quid->prop<MPtr(&property::PBag::get_any)>(prop_id)),
                                  user_data);
        else
          arglist = Py_BuildValue("(O)",
                                  pyInfoTree::any_to_pyobject(
                                      self->quid->prop<MPtr(&property::PBag::get_any)>(prop_id)));
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (!PyCallable_Check(cb)) {
    PyErr_SetString(PyExc_TypeError, "pyQuiddity callback argument must be callable");
    return nullptr;
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
  auto prop_id = self->quid->prop<MPtr(&property::PBag::get_id)>(prop_name);
  if (0 == prop_id) return false;
  auto found = self->prop_reg->props.find(prop_id);
  if (self->prop_reg->props.end() == found) return false;
  auto unsubscribed = self->quid->prop<MPtr(&property::PBag::unsubscribe)>(prop_id, found->second);
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
  auto sig_id = self->quid->sig<MPtr(&signal::SBag::get_id)>(signal_name);
  if (0 == sig_id) return false;
  auto found = self->sig_reg->signals.find(sig_id);
  if (self->sig_reg->signals.end() == found) return false;
  auto unsubscribed = self->quid->sig<MPtr(&signal::SBag::unsubscribe)>(sig_id, found->second);
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

PyDoc_STRVAR(pyquiddity_try_connect_doc,
             "Try to connect one of the follower to a writer of the quiddity.\n"
             "Arguments: (quid)\n"
             "Returns: a follower claw (FollowerClaw)\n");

PyObject* pyQuiddity::try_connect(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  PyObject* quid = nullptr;
  static char* kwlist[] = {(char*)"quid", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O", kwlist, &quid)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (quid && !PyObject_IsInstance(quid, reinterpret_cast<PyObject*>(&pyQuiddity::pyType))) {
    PyErr_SetString(PyExc_TypeError, "error quid argument is not a pyquid.Quiddidy instance");
    return nullptr;
  }

  const auto res = pyquid::ungiled(std::function([&]() {
    return self->quid->claw<MPtr(&Claw::try_connect)>(
        reinterpret_cast<pyQuiddity::pyQuiddityObject*>(quid)->quid->get_id());
  }));
  if (Ids::kInvalid == res) {
    PyErr_Format(PyExc_RuntimeError, "failed to connect, check switcher log for more information");
    return nullptr;
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
      PyUnicode_FromString(self->quid->claw<MPtr(&Claw::get_follower_label)>(res).c_str()));
  auto quid_capsule = PyCapsule_New(static_cast<void*>(self->quid), nullptr, nullptr);
  On_scope_exit { Py_XDECREF(quid_capsule); };
  PyDict_SetItemString(keyworded_args, "quid_c_ptr", quid_capsule);
  return PyObject_Call((PyObject*)&pyFollowerClaw::pyType, empty_tuple, keyworded_args);
}

PyDoc_STRVAR(pyquiddity_unsubscribe_doc,
             "Unsubscribe from a signal or a property.\n"
             "Arguments: (name)\n"
             "Returns: True or False\n");

PyObject* pyQuiddity::unsubscribe(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  const char* name = nullptr;
  static char* kwlist[] = {(char*)"name", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &name)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }

  if (unsubscribe_from_property(self, name)) {
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  return PyLong_FromLong(self->quid->sig<MPtr(&signal::SBag::get_id)>(signal));
}

PyObject* pyQuiddity::Quiddity_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyQuiddityObject* self;
  self = (pyQuiddityObject*)type->tp_alloc(type, 0);
  return (PyObject*)self;
}

int pyQuiddity::Quiddity_init(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyswitch = nullptr;
  char* kind = nullptr;
  char* nickname = nullptr;
  PyObject* pyinfotree = nullptr;

  static char* kwlist[] = {
      (char*)"switcher", (char*)"kind", (char*)"nickname", (char*)"config", nullptr};
  if (!PyArg_ParseTupleAndKeywords(
          args, kwds, "Os|sO", kwlist, &pyswitch, &kind, &nickname, &pyinfotree)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return -1;
  }
  if (pyswitch && !PyObject_IsInstance(pyswitch, reinterpret_cast<PyObject*>(&pySwitch::pyType))) {
    PyErr_SetString(PyExc_TypeError,
                    "error switcher argument is not an instance of a pyquid.Switcher");
    return -1;
  }

  if (pyinfotree &&
      !PyObject_IsInstance(pyinfotree, reinterpret_cast<PyObject*>(&pyInfoTree::pyType))) {
    PyErr_SetString(PyExc_TypeError,
                    "error config argument is not an instance of a pyquid.InfoTree");
    return -1;
  }

  // retrieve switcher instance
  auto pyswitchobj = reinterpret_cast<pySwitch::pySwitchObject*>(pyswitch);
  auto switcher = pyswitchobj->switcher;

  // create a quiddity without calling creation callbacks
  auto qrox = switcher->quids<MPtr(&quiddity::Container::quiet_create)>(
      kind,
      nickname ? nickname : std::string(),
      pyinfotree ? reinterpret_cast<pyInfoTree::pyInfoTreeObject*>(pyinfotree)->tree : nullptr);

  // try to retrieve a pointer to quiddity
  auto quid = qrox.get();

  // check for pointer to quiddity
  if (!quid) {
    PyErr_Format(PyExc_RuntimeError, "%s", qrox.msg().c_str());
    return -1;
  }

  self->quid = quid;
  self->sig_reg = std::make_unique<sig_registering_t>();
  self->prop_reg = std::make_unique<prop_registering_t>();
  self->async_invocations = std::make_unique<std::list<std::future<void>>>();

  // append quiddity into quiddities list
  PyObject* obj = reinterpret_cast<PyObject*>(self);
  PyList_Append(pyswitchobj->quiddities, obj);

  self->interpreter_state = PyThreadState_Get()->interp;
  PyEval_InitThreads();

  switcher->quids<MPtr(&quiddity::Container::notify_quiddity_created)>(quid);

  return 0; 
}

void pyQuiddity::Quiddity_dealloc(pyQuiddityObject* self) {
  if (self->quid) {
    // cleaning signal subscription
    for (const auto& it : self->sig_reg->callbacks) {
      const auto found = self->sig_reg->signals.find(it.first);
      self->quid->sig<MPtr(&signal::SBag::unsubscribe)>(found->first, found->second);
      Py_XDECREF(it.second);
    }
    for (auto& it : self->sig_reg->user_data) {
      Py_XDECREF(it.second);
    }
    self->sig_reg.reset();

    // cleaning prop subscription
    for (const auto& it : self->prop_reg->callbacks) {
      auto found = self->prop_reg->props.find(it.first);
      self->quid->prop<MPtr(&property::PBag::unsubscribe)>(found->first, found->second);
      Py_XDECREF(it.second);
    }
    for (auto& it : self->prop_reg->user_data) {
      Py_XDECREF(it.second);
    }
    self->prop_reg.reset();
    self->async_invocations.reset();
  }
  // cleaning self
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyDoc_STRVAR(pyquiddity_get_writer_claws_doc,
             "Get a list of all WriterClaw available for the Quiddity.\n"
             "Arguments: ()\n"
             "Returns: the list of WriterClaw objects\n");

PyObject* pyQuiddity::get_writer_claws(pyQuiddityObject* self, PyObject*, PyObject*) {
  auto labels = self->quid->claw<MPtr(&Claw::get_writer_labels)>();
  unsigned int i = 0;
  PyObject* result = PyList_New(labels.size());
  for (auto& label : labels) {
    // construct the Writer Claw object
    PyObject* empty_tuple = PyTuple_New(0);
    On_scope_exit { Py_XDECREF(empty_tuple); };
    PyObject* keyworded_args = PyDict_New();
    On_scope_exit { Py_XDECREF(keyworded_args); };
    Py_INCREF(Py_None);
    PyDict_SetItemString(keyworded_args, "quid", Py_None);
    PyDict_SetItemString(keyworded_args, "label", PyUnicode_FromString(label.c_str()));
    auto quid_capsule = PyCapsule_New(static_cast<void*>(self->quid), nullptr, nullptr);
    On_scope_exit { Py_XDECREF(quid_capsule); };
    PyDict_SetItemString(keyworded_args, "quid_c_ptr", quid_capsule);
    // add the object to the result
    PyList_SetItem(
        result, i, PyObject_Call((PyObject*)&pyWriterClaw::pyType, empty_tuple, keyworded_args));
    ++i;
  }
  return result;
}

PyDoc_STRVAR(pyquiddity_get_follower_claws_doc,
             "Get a list of all FollowerClaw available for the Quiddity.\n"
             "Arguments: ()\n"
             "Returns: the list of FollowerClaw objects\n");

PyObject* pyQuiddity::get_follower_claws(pyQuiddityObject* self, PyObject* args, PyObject* kwds) {
  auto labels = self->quid->claw<MPtr(&Claw::get_follower_labels)>();
  unsigned int i = 0;
  PyObject* result = PyList_New(labels.size());
  for (auto& label : labels) {
    // construct the Writer Claw object
    PyObject* empty_tuple = PyTuple_New(0);
    On_scope_exit { Py_XDECREF(empty_tuple); };
    PyObject* keyworded_args = PyDict_New();
    On_scope_exit { Py_XDECREF(keyworded_args); };
    Py_INCREF(Py_None);
    PyDict_SetItemString(keyworded_args, "quid", Py_None);
    PyDict_SetItemString(keyworded_args, "label", PyUnicode_FromString(label.c_str()));
    auto quid_capsule = PyCapsule_New(static_cast<void*>(self->quid), nullptr, nullptr);
    On_scope_exit { Py_XDECREF(quid_capsule); };
    PyDict_SetItemString(keyworded_args, "quid_c_ptr", quid_capsule);
    // add the object to the result
    PyList_SetItem(
        result, i, PyObject_Call((PyObject*)&pyFollowerClaw::pyType, empty_tuple, keyworded_args));
    ++i;
  }
  return result;
}

PyDoc_STRVAR(pyquiddity_get_connection_specs_doc,
             "Get connection specifications.\n"
             "Arguments: ()\n"
             "Returns: the connection specifications (pyquid.InfoTree)\n");

PyObject* pyQuiddity::get_connection_specs(pyQuiddityObject* self, PyObject*, PyObject*) {
  self->connnection_spec_keep_alive_ = self->quid->conspec<MPtr(&InfoTree::get_copy)>();
  return pyInfoTree::make_pyobject_from_c_ptr(self->connnection_spec_keep_alive_.get(), false);
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
    {"get_user_tree",
     (PyCFunction)pyQuiddity::get_user_tree,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_get_user_tree_doc},
    {"get_info",
     (PyCFunction)pyQuiddity::get_info,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_get_info_doc},
    {"nickname",
     (PyCFunction)pyQuiddity::nickname,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_nickname_doc},
    {"id", (PyCFunction)pyQuiddity::id, METH_VARARGS | METH_KEYWORDS, pyquiddity_id_doc},
    {"get_kind",
     (PyCFunction)pyQuiddity::get_kind,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_get_kind_doc},
    {"set_nickname",
     (PyCFunction)pyQuiddity::set_nickname,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_set_nickname_doc},
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
    {"try_connect",
     (PyCFunction)pyQuiddity::try_connect,
     METH_VARARGS | METH_KEYWORDS,
     pyquiddity_try_connect_doc},
    {"get_writer_claws",
     (PyCFunction)pyQuiddity::get_writer_claws,
     METH_NOARGS,
     pyquiddity_get_writer_claws_doc},
    {"get_follower_claws",
     (PyCFunction)pyQuiddity::get_follower_claws,
     METH_NOARGS,
     pyquiddity_get_follower_claws_doc},
    {"get_connection_specs",
     (PyCFunction)pyQuiddity::get_connection_specs,
     METH_NOARGS,
     pyquiddity_get_connection_specs_doc},
    {nullptr}};

PyDoc_STRVAR(pyquid_quiddity_doc,
             "Quiddity objects.\n"
             "Quiddities are services that can be instantiated by a switcher. "
             "Communication with a Quiddity is achieved through:\n"
             "   - property set, get and subscribe\n"
             "   - method invocation\n"
             "   - InfoTree request\n"
             "   - specific configuration\n\n"
             "Construct a Quiddity object:\n"
             "Arguments: (switcher, kind, name, config), where switcher is a pyquid.Switcher object"
             ", kind is a quiddity type (string), name is a nickname for the Quiddity (string)"
             "and config is an InfoTree that overrides the switcher configuration file for this "
             "Quiddity kind.\n"
             "Note: switcher and kind are mandatory arguments during construction of q Quiddity\n");

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

