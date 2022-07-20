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

#include "pybundle.hpp"

#include <atomic>

#include "pyswitch.hpp"

PyObject* BundleManager::tp_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
  BundleManagerObject* self;
  self = (BundleManagerObject*)type->tp_alloc(type, 0);
  return reinterpret_cast<PyObject*>(self);
}

int BundleManager::tp_init(BundleManagerObject* self, PyObject* args, PyObject* kwds) {
  PyObject* pyswitch = nullptr;
  if (!PyArg_ParseTuple(args, "O", &pyswitch)) return -1;

  self->pyswitch = pyswitch;
  self->registry = PyList_New(0);

  return 0;
}

void BundleManager::tp_dealloc(BundleManagerObject* self) {
  // retrieve registry list size
  auto rsize = PyList_Size(self->registry);
  // iterate over bundle list
  for (Py_ssize_t idx = 0; idx < rsize; idx++) {
    // borrowed reference to bundle
    PyObject* bundle = PyList_GetItem(self->registry, idx);
    // decrement refcount
    Py_XDECREF(bundle);
  }
  Py_XDECREF(self->registry);

  Py_TYPE(self)->tp_free((PyObject*)self);
}

// __get__
PyObject* BundleManager::tp_descr_get(BundleManagerObject* self, PyObject* args, PyObject* kwds) {
  return reinterpret_cast<PyObject*>(self);
}

PyObject* BundleManager::sq_length(BundleManagerObject* self) {
  return PyLong_FromSsize_t(PyList_Size(self->registry));
}

// __getitem__
PyObject* BundleManager::sq_item(BundleManagerObject* self, Py_ssize_t key) {
  // retrieve registry list size
  auto rsize = PyList_Size(self->registry);

  // iterate over bundle list
  for (Py_ssize_t idx = 0; idx < rsize; idx++) {
    // retrieve bundle from list
    PyObject* bundle = PyList_GetItem(self->registry, idx);
    // if key match index
    if (key == idx) {
      // increment bundle refcount
      Py_INCREF(bundle);
      // return bundle reference
      return bundle;
    }
  }

  // raise index error since there is no match
  PyErr_SetString(PyExc_IndexError, "list index out of range");
  return nullptr;
}

// __setitem__
PyObject* BundleManager::sq_ass_item(BundleManagerObject* self, PyObject* key, PyObject* value) {
  Py_ssize_t idx = PyLong_AsSsize_t(key);

  PyObject* BundleType = InterpType("Bundle", "bundles.py");

  // typecheck value
  if (!(value && PyObject_IsInstance(value, BundleType))) {
    PyErr_SetString(PyExc_TypeError, "value is not an instance of `Bundle`");
    return nullptr;
  }

  if (!PyList_SetItem(self->registry, idx, value)) return nullptr;

  Py_RETURN_NONE;
}

// append
PyObject* BundleManager::append(BundleManagerObject* self, PyObject* args, PyObject* kwds) {
  PyObject* bundle = nullptr;
  if (!PyArg_ParseTuple(args, "O", &bundle)) return nullptr;

  // append bundle into python registry
  // @NOTE: might be unecessary if we read a list of bundles
  // from the configuration instead so that we get actually
  // registered bundles and not a synchronized copy
  if (PyList_Append(self->registry, bundle) == -1) return nullptr;

  // call str(bundle)
  auto builtins = PyEval_GetBuiltins();
  auto func = PyDict_GetItemString(builtins, "str");
  auto bundle_json = PyObject_CallFunctionObjArgs(func, bundle, nullptr);

  // deserialize dumped json to an infotree
  auto bundle_tree = infotree::json::deserialize(PyUnicode_AsUTF8(bundle_json));

  // register bundle into configuration
  auto switcher = reinterpret_cast<pySwitch::pySwitchObject*>(self->pyswitch)->switcher;
  switcher->register_bundle(bundle_tree);

  // decrement refcount
  Py_XDECREF(bundle_json);

  Py_RETURN_NONE;
}

// create
PyObject* BundleManager::create(BundleManagerObject* self, PyObject* args, PyObject* kwds) {
  PyObject *name = nullptr, *pipeline = nullptr, *doc = nullptr;
  static char* kwlist[] = {(char*)"name", (char*)"pipeline", (char*)"doc", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "UUO", kwlist, &name, &pipeline, &doc))
    return nullptr;

  // doc typecheck
  if (doc && !PyDict_Check(doc)) return nullptr;

  // retrieve bundle type
  PyObject* BundleType = InterpType("Bundle", "bundles.py");

  // init bundle
  PyObject* bundle = PyObject_CallFunctionObjArgs(BundleType, name, pipeline, doc, nullptr);
  if (bundle) {
    // call opts
    PyObject *mngr = reinterpret_cast<PyObject*>(self), *meth = PyUnicode_FromString("append");

    // register bundle
    PyObject_CallMethodObjArgs(mngr, meth, bundle, nullptr);

    // decrement refcount
    Py_XDECREF(meth);

    // return registered bundle
    Py_INCREF(bundle);
    return bundle;
  }

  Py_RETURN_NONE;
}

// pop
PyObject* BundleManager::pop(BundleManagerObject* self, PyObject* args, PyObject* kwds) {
  Py_ssize_t key = 0;
  if (!PyArg_ParseTuple(args, "n", &key)) return nullptr;

  // ensure list is not empty
  if (!PyList_Size(self->registry)) {
    PyErr_SetString(PyExc_IndexError, "pop from empty list");
    return nullptr;
  }

  // retrieve bundle to pop from list
  PyObject* bundle = nullptr;
  if (!(bundle = PyList_GetItem(self->registry, key))) return nullptr;

  // set slice to remove bundle from list
  if (!PyList_SetSlice(self->registry, key, key + 1, nullptr)) return nullptr;

  auto builtins = PyEval_GetBuiltins();
  auto func = PyDict_GetItemString(builtins, "str");
  auto bundle_json = PyObject_CallFunctionObjArgs(func, bundle, nullptr);

  // deserialize dumped json to an infotree
  auto bundle_tree = infotree::json::deserialize(PyUnicode_AsUTF8(bundle_json));

  // unregister bundle from configuration
  auto switcher = reinterpret_cast<pySwitch::pySwitchObject*>(self->pyswitch)->switcher;
  switcher->unregister_bundle(bundle_tree);

  // decrement refcount
  Py_XDECREF(bundle_json);

  // return popped bundle
  return bundle;
}

PyObject* BundleManager::tp_repr(BundleManagerObject* self, PyObject* args, PyObject* kwds) {
  // compute representation string
  std::ostringstream oss;
  oss << "<BundleManager ";

  // call to str(names)
  auto builtins = PyEval_GetBuiltins();
  auto func = PyDict_GetItemString(builtins, "str");
  auto str_names = PyObject_CallFunctionObjArgs(func, self->registry, nullptr);

  // append str for list of names to string stream
  oss << PyUnicode_AsUTF8(str_names) << ">";

  // decrement refcount
  Py_XDECREF(str_names);

  // return unicode from string stream
  return PyUnicode_FromString(oss.str().c_str());
}

PySequenceMethods BundleManager::tp_as_sequence = {.sq_length = (lenfunc)sq_length,
                                                   .sq_item = (ssizeargfunc)sq_item,
                                                   .sq_ass_item = (ssizeobjargproc)sq_ass_item};

PyMethodDef BundleManager::tp_methods[] = {
    {"append",
     (PyCFunction)BundleManager::append,
     METH_VARARGS,
     PyDoc_STR("Append a Bundle into the registry.\n\n"
               "Arguments:\n"
               "----------\n"
               "bundle: Bundle\n"
               "\tThe bundle to register.\n")},
    {"create",
     (PyCFunction)BundleManager::create,
     METH_VARARGS | METH_KEYWORDS,
     PyDoc_STR("Create and register a new bundle.\n\n"
               "Arguments:\n"
               "----------\n"
               "name: str\n"
               "\tThe name of the bundle.\n"
               "pipeline: str\n"
               "\tThe pipeline command line string.\n"
               "doc: dict\n"
               "\tThe documentation that specifies "
               "`long_name`, `category`, `tags` and"
               " a `description`.\n")},
    {"pop",
     (PyCFunction)BundleManager::pop,
     METH_VARARGS,
     PyDoc_STR("Remove a bundle at `index` or the first one if not specified.\n"
               "Raises an IndexError exception in case `index` is out of bounds.\n\n"
               "Arguments:\n"
               "----------\n"
               "index: int\n"
               "\tThe index of the bundle to remove\n")},
    {nullptr}  // Sentinel
};

PyTypeObject BundleManager::pyType = {PyVarObject_HEAD_INIT(nullptr, 0).tp_name = "BundleManager",
                                      .tp_basicsize = sizeof(BundleManagerObject),
                                      .tp_dealloc = (destructor)tp_dealloc,
                                      .tp_repr = (reprfunc)tp_repr,
                                      .tp_as_sequence = &tp_as_sequence,
                                      .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
                                      .tp_doc = ("The Bundle Manager for Switcher"),
                                      .tp_methods = tp_methods,
                                      .tp_init = (initproc)tp_init,
                                      .tp_new = (newfunc)tp_new};
