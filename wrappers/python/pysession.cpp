/*
 * This file is part of libswitcher.
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
 *
 * @file pysession.hpp
 * @brief The `Session Manager` for Switcher.
 *        Allows to create new session files based on `session.path` configuration
 *        key and to copy, list, remove and read existing session files.
 *
 * @author Aurélien Perronneau
 * @date 2021-08-14
 */

#include "./pyswitch.hpp"

PyObject* pySession::tp_new(PyTypeObject* type,
                            PyObject* Py_UNUSED(args),
                            PyObject* Py_UNUSED(kwargs)) {
  SessionObject* self;
  self = (SessionObject*)type->tp_alloc(type, 0);
  return reinterpret_cast<PyObject*>(self);
}

void pySession::tp_dealloc(SessionObject* self,
                           PyObject* Py_UNUSED(args),
                           PyObject* Py_UNUSED(kwargs)) {
  Py_XDECREF(self->pyswitch);
  Session::ptr empty;
  self->csession.swap(empty);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

int pySession::tp_init(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs)) {
  // parse arguments
  PyObject* pyswitch = nullptr;
  if (!PyArg_ParseTuple(args, "O", &pyswitch)) {
    PyErr_SetString(PyExc_TypeError, "error parsing arguments");
    return -1;
  }
  // type check for `pyswitch`
  if (pyswitch && !PyObject_IsInstance(pyswitch, reinterpret_cast<PyObject*>(&pySwitch::pyType))) {
    PyErr_SetString(PyExc_TypeError, "TypeError: argument is not an instance of `pyquid.Switcher`");
    return -1;
  }

  self->pyswitch = pyswitch;
  Py_INCREF(self->pyswitch);
  auto pyswitch_obj = reinterpret_cast<pySwitch::pySwitchObject*>(pyswitch);
  // initialize csession for switcher
  self->csession = pyswitch_obj->switcher.get()->session;
  // set interpreter state
  self->interpreter_state = PyThreadState_Get()->interp;
  return 0;
}

PyObject* pySession::save_as(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs)) {
  const char* filename = nullptr;
  if (!PyArg_ParseTuple(args, "s", &filename)) return nullptr;
  // call C++ session method
  const std::string result = self->csession->save_as(filename);
  if (result.empty())
    return nullptr;
  else
    return PyUnicode_FromString(result.c_str());
};

PyObject* pySession::copy(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs)) {
  const char *src = nullptr, *dst = nullptr;
  // parse positional and keyword arguments
  if (!PyArg_ParseTuple(args, "ss", &src, &dst)) return nullptr;
  // call C++ session method and return a python boolean
  if(self->csession->copy(src, dst))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
};

PyObject* pySession::list(SessionObject* self,
                          PyObject* Py_UNUSED(args),
                          PyObject* Py_UNUSED(kwargs)) {
  // new python list
  PyObject* session_files = PyList_New(0);
  // append each directory entry from the session list iterator in the python list
  for (auto& entry : self->csession->list())
    PyList_Append(session_files, Py_BuildValue("s", entry.path().filename().c_str()));
  // return the python list of session filenames
  return session_files;
};

PyObject* pySession::remove(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs)) {
  const char* filename = nullptr;
  if (!PyArg_ParseTuple(args, "s", &filename)) return nullptr;
  // call C++ session method and return a python boolean
  if(self->csession->remove(filename))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
};

PyObject* pySession::load(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs)) {
  const char* filename = nullptr;
  if (!PyArg_ParseTuple(args, "s", &filename)) return nullptr;
  // call C++ session method and return a python boolean
  if(self->csession->load(filename))
    Py_RETURN_TRUE;
  else {
    PyErr_SetString(PyExc_RuntimeError, "Switcher could not load session file");
    Py_RETURN_FALSE;
  }
};

PyObject* pySession::read(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs)) {
  const char* filename = nullptr;
  if (!PyArg_ParseTuple(args, "s", &filename)) return nullptr;

  // call C++ session read method and return a python string
  auto session_content = self->csession->read(filename);

  if (session_content.empty()) {
    PyErr_SetString(PyExc_RuntimeError, "Switcher could not read session file");
    return nullptr;
  } else {
    return PyUnicode_FromString(session_content.c_str());
  }
};

PyObject* pySession::write(SessionObject* self, PyObject* args, PyObject* Py_UNUSED(kwargs)) {
  const char* session_content = nullptr;
  const char* filename = nullptr;
  if (!PyArg_ParseTuple(args, "ss", &session_content, &filename)) return nullptr;

  // call C++ session write method and return a python boolean
  if(self->csession->write(session_content, filename))
    Py_RETURN_TRUE;
  else {
    PyErr_SetString(PyExc_RuntimeError, "Switcher could not write session file");
    Py_RETURN_FALSE;
  }
};

PyMethodDef pySession::methods[] = {
    {"copy",
     (PyCFunction)pySession::copy,
     METH_VARARGS,
     PyDoc_STR("Duplicate an existing session file with a new name.\n"
               "Does not have to be the current session file.\n\n"
               "Arguments:\n"
               "----------\n"
               "src: str\n"
               "\tThe name of the source session file to copy.\n"
               "dst: str\n"
               "\tThe name of the destination file to copy the source to\n\n"
               "Returns: A boolean asserting how the copy went.\n")},
    {"list",
     (PyCFunction)pySession::list,
     METH_NOARGS,
     PyDoc_STR("List the existing session files in the session directory.\n\n"
               "Returns: A list of existing session files.\n")},
    {"load",
     (PyCFunction)pySession::load,
     METH_VARARGS,
     PyDoc_STR("Load a session file on disk, parse it "
               "and set Switcher's state from it.\n\n"
               "Arguments:\n"
               "----------\n"
               "filename: str\n"
               "\tThe name of the session file to load\n\n"
               "Returns: A boolean asserting how the load went.\n")},
    {"read",
     (PyCFunction)pySession::read,
     METH_VARARGS,
     PyDoc_STR("Read the contents of a session file on disk, and return content\n\n"
               "Arguments:\n"
               "----------\n"
               "filename: str\n"
               "\tThe name of the session file to read\n\n"
               "Returns: content of session file.\n")},
    {"write",
     (PyCFunction)pySession::write,
     METH_VARARGS,
     PyDoc_STR("Write content to a session file on disk without affecting current session state\n\n"
               "Arguments:\n"
               "----------\n"
               "content: str\n"
               "\tContent to write to session file\n\n"
               "filename: str\n"
               "\tThe name of the session file to write to\n\n"
               "Returns: A boolean asserting how the write went.\n")},
    {"remove",
     (PyCFunction)pySession::remove,
     METH_VARARGS,
     PyDoc_STR("Delete an existing session file without affecting "
               "the current session state.\n\n"
               "Arguments:\n"
               "----------\n"
               "filename: str\n"
               "\tThe name of the session file to remove\n\n"
               "Returns: A boolean asserting how the removal went.\n")},
    {"save_as",
     (PyCFunction)pySession::save_as,
     METH_VARARGS,
     PyDoc_STR("Write Switcher's current state into a new file.\n"
               "Arguments:\n"
               "----------\n"
               "filename: str\n"
               "\tThe name of the file to save the session to\n\n"
               "Returns: Either an empty string or the path to the saved session file.\n")},
    {nullptr}  // sentinel
};

PyTypeObject pySession::pyType = {
    PyVarObject_HEAD_INIT(NULL, 0).tp_name = "Session",
    .tp_basicsize = sizeof(SessionObject),
    .tp_itemsize = 0,
    .tp_dealloc = (destructor)tp_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = ("The `Session Manager` for Switcher.\n"
               "Allows to create new session files based on `session.path` configuration\n"
               "key and to copy, list, remove and read existing session files.\n"),
    .tp_methods = methods,
    .tp_init = (initproc)tp_init,
    .tp_new = tp_new,
};
