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

#include <switcher/infotree/json-serializer.hpp>
#include <switcher/utils/scope-exit.hpp>

PyObject* pyInfoTree::InfoTree_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyInfoTreeObject* self = reinterpret_cast<pyInfoTreeObject*>(type->tp_alloc(type, 0));
  return reinterpret_cast<PyObject*>(self);
}

int pyInfoTree::InfoTree_init(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  // initialize default values for params
  PyObject* initial = nullptr;
  // define keyword params list
  static char* kwlist[] = {(char*)"initial", nullptr};
  // parse params
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|O", kwlist, &initial)) return -1;

  // init from empty args
  if (initial == nullptr) {
    self->keepAlive = infotree::json::deserialize("{}");
    self->tree = self->keepAlive.get();
    return 0;
  }

  // init from dict
  if (PyDict_Check(initial)) {
    // function call objects
    PyObject *obj = PyImport_ImportModule("json"), *method = nullptr, *res = nullptr;
    // call options
    method = PyUnicode_FromString("dumps");
    // call function
    res = PyObject_CallMethodObjArgs(obj, method, initial, NULL);
    // deserialize json string and get a shared pointer
    self->keepAlive = infotree::json::deserialize(PyUnicode_AsUTF8(res));
    self->tree = self->keepAlive.get();
    // decrement refcounts
    for (auto& o : {obj, method, res}) Py_XDECREF(o);
    return 0;
  }

  // init from json string
  if (PyUnicode_Check(initial)) {
    // init infotree from json string
    self->keepAlive = infotree::json::deserialize(initial ? PyUnicode_AsUTF8(initial) : "{}");
    self->tree = self->keepAlive.get();
    // decrement refcounts
    // handle possible parsing error
    for (auto& o : {obj, method, res}) Py_XDECREF(o);
    if (self->keepAlive->branch_has_data(".parsing_error")) {
      PyErr_Format(PyExc_RuntimeError,
                   "parsing error: %s",
                   self->tree->branch_get_value(".parsing_error").copy_as<std::string>().c_str());
      return -1;
    }
    return 0;
  };

  // raise type error
  PyErr_SetString(PyExc_TypeError,
                  "function takes either a dictionary or a json string as its first optional "
                  "argument `initial`.");
  return -1;
}

void pyInfoTree::InfoTree_dealloc(pyInfoTreeObject* self) {
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyDoc_STRVAR(pyinfotree_empty_doc,
             "Test is the tree is empty.\n"
             "Arguments: ()\n"
             "Returns: True or False\n");

PyObject* pyInfoTree::empty(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  if (!self->tree->empty())
    Py_RETURN_FALSE;
  else
    Py_RETURN_TRUE;
}

PyDoc_STRVAR(pyinfotree_prune_doc,
             "Remove the subtree.\n"
             "Arguments: (path)\n"
             "Returns: True if branch has been pruned\n");

PyObject* pyInfoTree::prune(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &path)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  auto res = self->tree->prune(path);
  if (!res) {
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
  }
  if (nullptr == path) path = ".";
  auto tree = self->tree->get_tree(path);
  return pyInfoTree::make_pyobject_from_c_ptr(tree.get(), true);
}

PyDoc_STRVAR(pyinfotree_graft_doc,
             "Graft a value or another InfoTree instance.\n"
             "Arguments: (path, value)\n"
             "Returns: True or False\n");

PyObject* pyInfoTree::graft(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = nullptr;
  PyObject* val = 0;
  static char* kwlist[] = {(char*)"path", (char*)"value", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "sO", kwlist, &path, &val)) return nullptr;

  if (PyBool_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyObject_IsTrue(val) ? true : false))) {
      Py_RETURN_FALSE;
    }
  } else if (PyLong_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyLong_AsLong(val)))) {
      Py_RETURN_FALSE;
    }
  } else if (PyFloat_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyFloat_AsDouble(val)))) {
      Py_RETURN_FALSE;
    }
  } else if (PyUnicode_Check(val)) {
    if (!self->tree->graft(path, InfoTree::make(PyUnicode_AsUTF8(val)))) {
      Py_RETURN_FALSE;
    }
  } else if (PyDict_Check(val) || PyTuple_Check(val) || PyList_Check(val)) {
    PyObject *obj = PyImport_ImportModule("json"), *method = PyUnicode_FromString("dumps");
    PyObject* res = PyObject_CallMethodObjArgs(obj, method, val, nullptr);
    auto tree = infotree::json::deserialize(PyUnicode_AsUTF8(res));
    // decrement refcounts
    for (auto& o : {obj, method, res}) Py_XDECREF(o);
    // graft tree from dumped json
    if (!self->tree->graft(path, tree)) return nullptr;
  } else if (PyObject_IsInstance(val, reinterpret_cast<PyObject*>(&pyInfoTree::pyType))) {
    if (!self->tree->graft(path, reinterpret_cast<pyInfoTree::pyInfoTreeObject*>(val)->keepAlive)) {
      Py_RETURN_FALSE;
    }
  } else {  // unsorted PyObject type
    Py_RETURN_FALSE;
  }
  // return true if everything went well
  Py_RETURN_TRUE;
}

PyDoc_STRVAR(pyinfotree_json_doc,
             "Get a JSON representation of the tree.\n"
             "Arguments: (path)\n"
             "Returns: The JSON representation of the tree\n");

PyObject* pyInfoTree::json(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = ".";
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", kwlist, &path)) return nullptr;
  // serialize infotree to a json string
  auto str = infotree::json::serialize(self->tree->get_tree(path).get());
  return PyUnicode_FromString(str.c_str());
}

PyObject* pyInfoTree::any_to_pyobject(const Any& any) {
  auto category = any.get_category();
  if (AnyCategory::NONE == category) {  // any is empty
    Py_RETURN_NONE;
  } else if (AnyCategory::BOOLEAN == category) {
    if (!any.as<bool>()) Py_RETURN_FALSE;
    Py_RETURN_TRUE;
  } else if (AnyCategory::INTEGRAL == category) {
    return PyLong_FromLong(any.copy_as<long>());
  } else if (AnyCategory::FLOATING_POINT == category) {
    return PyFloat_FromDouble(any.copy_as<double>());
  } else if (AnyCategory::OTHER == category) {
    return PyUnicode_FromString(Any::to_string(any).c_str());
  }
  // should not happen since all possible values of AnyCategory are checked
  Py_RETURN_NONE;
}

PyDoc_STRVAR(pyinfotree_get_doc,
             "Get value at path (defaults to root node \".\")."
             "Arguments: (path)\n"
             "Returns: value\n");

PyObject* pyInfoTree::get(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  const char* path = ".";
  static char* kwlist[] = {(char*)"path", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", kwlist, &path)) return nullptr;

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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
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
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return nullptr;
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

PyObject* pyInfoTree::tp_str(pyInfoTreeObject* self) {
  // serialize infotree as a json encoded string
  auto str = infotree::json::serialize(self->tree->get_tree(".").get());
  // return new ref to python string
  return PyUnicode_FromString(str.c_str());
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
             "An InfoTree object can be serialized using the str() built-in function.\n");

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
    (reprfunc)tp_str,                                           /* tp_str */
    PyObject_GenericGetAttr,                                    /* tp_getattro */
    PyObject_GenericSetAttr,                                    /* tp_setattro */
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
    (newfunc)InfoTree_new                                       /* tp_new */
};

// pyInfoTree C++ only methods

// set C pointer to an infotree for a given pyinfotree instance
void pyInfoTree::set_tree(PyObject* instance, InfoTree* tree, bool copy) {
  pyInfoTreeObject* self = reinterpret_cast<pyInfoTreeObject*>(instance);
  self->keepAlive = copy ? InfoTree::copy(tree) : InfoTree::ptr(tree);
  self->tree = self->keepAlive.get();
};

// init pyinfotree instance from a C pointer to an infotree
PyObject* pyInfoTree::make_pyobject_from_c_ptr(InfoTree* tree, bool copy) {
  // call options
  PyObject* type = reinterpret_cast<PyObject*>(&pyInfoTree::pyType);
  // call type without options
  PyObject* obj = PyObject_CallFunctionObjArgs(type, nullptr);
  // set c pointer for obj instance
  pyInfoTree::set_tree(obj, tree, copy);
  return obj;
}
