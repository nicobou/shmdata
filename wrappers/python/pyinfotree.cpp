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
#include <switcher/information-tree-json.hpp>

PyObject* pyInfoTree::InfoTree_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyInfoTreeObject* self;
  self = (pyInfoTreeObject*)type->tp_alloc(type, 0);
  return (PyObject*)self;
}

int pyInfoTree::InfoTree_init(pyInfoTreeObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyinfotree;
  int copy;
  static char* kwlist[] = {(char*)"infotree_c_ptr", (char*)"copy", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|p", kwlist, &pyinfotree, &copy)) return -1;
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
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "s", kwlist, &path)) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  auto tree = self->tree->get_tree(path);

  auto tree_capsule = PyCapsule_New(static_cast<void*>(tree.get()), nullptr, nullptr);
  PyObject* argList = Py_BuildValue("Oi", tree_capsule, true);
  PyObject* obj = PyObject_CallObject((PyObject*)&pyInfoTree::pyType, argList);
  Py_XDECREF(argList);
  Py_XDECREF(tree_capsule);
  return obj;
}

PyDoc_STRVAR(pyinfotree_graft_doc,
             "Graft value (Integer or float or bool or string).\n"
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

  auto res = self->tree->branch_get_value(path);
  auto category = res.get_category();
  if (AnyCategory::NONE == category) {  // res is empty
    Py_INCREF(Py_None);
    return Py_None;
  } else if (AnyCategory::BOOLEAN == category) {
    if (!res.as<bool>()) {
      Py_INCREF(Py_False);
      return Py_False;
    }
    Py_INCREF(Py_True);
    return Py_True;
  } else if (AnyCategory::INTEGRAL == category) {
    return PyLong_FromLong(res.as<long>());
  } else if (AnyCategory::FLOATING_POINT == category) {
    return PyFloat_FromDouble(res.as<double>());
  } else if (AnyCategory::OTHER == category) {
    return PyUnicode_FromString(Any::to_string(res).c_str());
  }
  // should not happen since all possible values of AnyCategory are checked
  Py_INCREF(Py_None);
  return Py_None;
}

PyMethodDef pyInfoTree::pyInfoTree_methods[] = {
    {"prune", (PyCFunction)prune, METH_VARARGS | METH_KEYWORDS, pyinfotree_prune_doc},
    {"copy", (PyCFunction)copy, METH_VARARGS | METH_KEYWORDS, pyinfotree_copy_doc},
    {"graft", (PyCFunction)graft, METH_VARARGS | METH_KEYWORDS, pyinfotree_graft_doc},
    {"json", (PyCFunction)json, METH_VARARGS | METH_KEYWORDS, pyinfotree_json_doc},
    {"get", (PyCFunction)get, METH_VARARGS | METH_KEYWORDS, pyinfotree_get_doc},
    {nullptr}};

PyDoc_STRVAR(pyquid_pyinfotree_doc,
             "InfoTree objects.\n"
             "A InfoTree is a key value structure where keys are string based, hierarchical and "
             "separated by dots, e.g. \"root.child.subchild\". Values are typed and the tree can "
             "be serialized & deserialized.\n");

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
