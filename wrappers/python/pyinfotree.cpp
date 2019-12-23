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

#include "./pyinfotree.hpp"
#include <switcher/infotree/information-tree-json.hpp>
#include <switcher/utils/scope-exit.hpp>

PyObject* pyInfoTree::InfoTree_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyInfoTreeObject* self;
  self = (pyInfoTreeObject*)type->tp_alloc(type, 0);
  return (PyObject*)self;
}

int pyInfoTree::InfoTree_init(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyinfotree = nullptr;
  int copy = 0;
  const char* json_descr = nullptr;
  static char* kwlist[] = {(char*)"json", (char*)"infotree_c_ptr", (char*)"copy", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|sOp", kwlist, &json_descr, &pyinfotree, &copy))
    return -1;
  if (!pyinfotree && !json_descr) json_descr = "null";
  if (nullptr != json_descr) {  // build a tree from the json description
    self->keepAlive = JSONSerializer::deserialize(json_descr);
    self->tree = self->keepAlive.get();
    return 0;
  }
  if (copy) {
    self->keepAlive =
        InfoTree::copy(static_cast<InfoTree*>(PyCapsule_GetPointer(pyinfotree, nullptr)));
    self->tree = self->keepAlive.get();
  } else {
    self->tree = static_cast<InfoTree*>(PyCapsule_GetPointer(pyinfotree, nullptr));
  }

  return 0;
}

void pyInfoTree::InfoTree_dealloc(pyInfoTreeObject* self) {
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyDoc_STRVAR(pyinfotree_empty_doc,
             "Test is the tree is empty.\n"
             "Arguments: ()\n"
             "Returns: True or False\n");

PyObject* pyInfoTree::empty(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  if (!self->tree->empty()) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyinfotree_prune_doc,
             "Remove the subtree.\n"
             "Arguments: (path)\n"
             "Returns: None\n");

PyObject* pyInfoTree::prune(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto res = self->tree->prune(path);
  if (res->empty()) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyinfotree_copy_doc,
             "Copy the subtree.\n"
             "Arguments: (path)\n"
             "Returns: A copy of the subtree\n");

PyObject* pyInfoTree::copy(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (nullptr == path) path = ".";
  auto tree = self->tree->get_tree(path);
  return pyInfoTree::make_pyobject_from_c_ptr(tree.get(), true);
}

PyDoc_STRVAR(pyinfotree_graft_doc,
             "Graft a value (Integer or float or bool or string) or another InfoTree.\n"
             "Arguments: (path, value)\n"
             "Returns: True or False\n");

PyObject* pyInfoTree::graft(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  PyObject* val = 0;
  static char* kwlist[] = {(char*)"path", (char*)"value", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO", kwlist, &path, &val)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  if (PyBool_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyObject_IsTrue(val) ? true : false))) {
      Py_INCREF(Py_False);
      return Py_False;
    }
  } else if (PyLong_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyLong_AsLong(val)))) {
      Py_INCREF(Py_False);
      return Py_False;
    }
  } else if (PyFloat_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyFloat_AsDouble(val)))) {
      Py_INCREF(Py_False);
      return Py_False;
    }
  } else if (PyUnicode_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyUnicode_AsUTF8(val)))) {
      Py_INCREF(Py_False);
      return Py_False;
    }
  } else if (PyObject_IsInstance(val, reinterpret_cast<PyObject*>(&pyInfoTree::pyType))) {
    if (!self->tree->graft(path, reinterpret_cast<pyInfoTree::pyInfoTreeObject*>(val)->keepAlive)) {
      Py_INCREF(Py_False);
      return Py_False;
    }
  } else {  // unsorted PyObject type
    Py_INCREF(Py_False);
    return Py_False;
  }
  // return true if everything went well
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyinfotree_json_doc,
             "Get a JSON representation of the tree.\n"
             "Arguments: (path)\n"
             "Returns: the JSON representation \n");

PyObject* pyInfoTree::json(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (nullptr == path) path = ".";
  return PyUnicode_FromString(JSONSerializer::serialize(self->tree->get_tree(path).get()).c_str());
}

PyObject* pyInfoTree::any_to_pyobject(const Any& any) {
  auto category = any.get_category();
  if (AnyCategory::NONE == category) {  // any is empty
    Py_INCREF(Py_None);
    return Py_None;
  } else if (AnyCategory::BOOLEAN == category) {
    if (!any.as<bool>()) {
      Py_INCREF(Py_False);
      return Py_False;
    }
    Py_INCREF(Py_True);
    return Py_True;
  } else if (AnyCategory::INTEGRAL == category) {
    return PyLong_FromLong(any.copy_as<long>());
  } else if (AnyCategory::FLOATING_POINT == category) {
    return PyFloat_FromDouble(any.copy_as<double>());
  } else if (AnyCategory::OTHER == category) {
    return PyUnicode_FromString(Any::to_string(any).c_str());
  }
  // should not happen since all possible values of AnyCategory are checked
  Py_INCREF(Py_None);
  return Py_None;
}

PyDoc_STRVAR(pyinfotree_get_doc,
             "Get value at path.\n"
             "Arguments: (path)\n"
             "Returns: value\n");

PyObject* pyInfoTree::get(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  return any_to_pyobject(self->tree->branch_get_value(path));
}

PyDoc_STRVAR(pyinfotree_tag_as_array_doc,
             "Tag a branch as an array, according to the value of the is_array argument (True by "
             "default). \n"
             "Arguments: (path, is_array)\n"
             "Returns: True or False\n");

PyObject* pyInfoTree::tag_as_array(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  int is_array = 1;
  static char* kwlist[] = {(char*)"path", (char*)"is_array", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|p", kwlist, &path, &is_array)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (!self->tree->tag_as_array(path, is_array ? true : false)) {
    Py_INCREF(Py_False);
    return Py_False;
  }
  Py_INCREF(Py_True);
  return Py_True;
}

PyDoc_STRVAR(pyinfotree_get_child_keys_doc,
             "Get a list of child keys. \n"
             "Arguments: (path)\n"
             "Returns: a list of keys\n");

PyObject* pyInfoTree::get_child_keys(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  if (nullptr == path) path = ".";
  auto keys = self->tree->get_child_keys(path);
  unsigned int i = 0;
  PyObject* result = PyList_New(keys.size());
  for (auto& it : keys) {
    PyList_SetItem(result, i, Py_BuildValue("s", it.c_str()));
    ++i;
  }
  return result;
}

PyDoc_STRVAR(pyinfotree_get_key_values_doc,
             "Get a list of values for a given key. The sibling argument allows for selecting "
             "if the search continues with sibling of a found key. Default is True\n"
             "Arguments: (key, sibling)\n"
             "Returns: a list of values\n");

PyObject* pyInfoTree::get_key_values(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* key = nullptr;
  int sibling = 1;
  static char* kwlist[] = {(char*)"key", (char*)"sibling", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|i", kwlist, &key, &sibling)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto collected = InfoTree::collect_values(
      self->tree,
      [&](const std::string& keyarg, InfoTree::ptrc) { return keyarg == key; },
      sibling);

  unsigned int i = 0;
  PyObject* result = PyList_New(collected.size());
  for (auto& it : collected) {
    PyList_SetItem(result, i, any_to_pyobject(it));
    ++i;
  }
  return result;
}

PyMethodDef pyInfoTree::pyInfoTree_methods[] = {
    {"empty", (PyCFunction)empty, METH_VARARGS | METH_KEYWORDS, pyinfotree_empty_doc},
    {"prune", (PyCFunction)prune, METH_VARARGS | METH_KEYWORDS, pyinfotree_prune_doc},
    {"copy", (PyCFunction)copy, METH_VARARGS | METH_KEYWORDS, pyinfotree_copy_doc},
    {"graft", (PyCFunction)graft, METH_VARARGS | METH_KEYWORDS, pyinfotree_graft_doc},
    {"json", (PyCFunction)json, METH_VARARGS | METH_KEYWORDS, pyinfotree_json_doc},
    {"get", (PyCFunction)get, METH_VARARGS | METH_KEYWORDS, pyinfotree_get_doc},
    {"get_child_keys",
     (PyCFunction)get_child_keys,
     METH_VARARGS | METH_KEYWORDS,
     pyinfotree_get_child_keys_doc},
    {"get_key_values",
     (PyCFunction)get_key_values,
     METH_VARARGS | METH_KEYWORDS,
     pyinfotree_get_key_values_doc},
    {"tag_as_array",
     (PyCFunction)tag_as_array,
     METH_VARARGS | METH_KEYWORDS,
     pyinfotree_tag_as_array_doc},
    {nullptr}};

PyDoc_STRVAR(pyquid_pyinfotree_doc,
             "InfoTree objects.\n"
             "A InfoTree is a key value structure where keys are string based, hierarchical and "
             "separated by dots, e.g. \"root.child.subchild\". Values are typed and the tree can "
             "be serialized & deserialized.\n"
             "An InfoTree object can be serialized using the json() method, and deserialized using "
             "the InfoTree constructor with the json description given as argument\n");

PyTypeObject pyInfoTree::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyquid.InfoTree", /* tp_name */
    sizeof(pyInfoTreeObject),                                   /* tp_basicsize */
    0,                                                          /* tp_itemsize */
    (destructor)InfoTree_dealloc,                               /* tp_dealloc */
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
    pyquid_pyinfotree_doc,                                      /* tp_doc */
    0,                                                          /* tp_traverse */
    0,                                                          /* tp_clear */
    0,                                                          /* tp_richcompare */
    0,                                                          /* tp_weaklistoffset */
    0,                                                          /* tp_iter */
    0,                                                          /* tp_iternext */
    pyInfoTree_methods,                                         /* tp_methods */
    0,                                                          /* tp_members */
    0,                                                          /* tp_getset */
    0,                                                          /* tp_base */
    0,                                                          /* tp_dict */
    0,                                                          /* tp_descr_get */
    0,                                                          /* tp_descr_set */
    0,                                                          /* tp_dictoffset */
    (initproc)InfoTree_init,                                    /* tp_init */
    0,                                                          /* tp_alloc */
    InfoTree_new                                                /* tp_new */
};

PyObject* pyInfoTree::make_pyobject_from_c_ptr(InfoTree* tree, bool do_copy) {
  PyObject* keyworded_args = PyDict_New();
  On_scope_exit { Py_XDECREF(keyworded_args); };
  PyObject* empty_tuple = PyTuple_New(0);
  On_scope_exit { Py_XDECREF(empty_tuple); };
  auto tree_capsule = PyCapsule_New(static_cast<void*>(tree), nullptr, nullptr);
  On_scope_exit { Py_XDECREF(tree_capsule); };
  PyDict_SetItemString(keyworded_args, "infotree_c_ptr", tree_capsule);
  PyObject* copy = PyLong_FromLong(do_copy);
  On_scope_exit { Py_XDECREF(copy); };
  PyDict_SetItemString(keyworded_args, "copy", copy);
  PyObject* obj = PyObject_Call((PyObject*)&pyInfoTree::pyType, empty_tuple, keyworded_args);
  return obj;
}
