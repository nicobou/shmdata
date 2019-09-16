/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#include "pyshmdata.h"
#include <string>

using namespace std;

/*************/
void log_error_handler(void* /*user_data*/, const char* str) { printf("Error: %s\n", str); }

/*************/
void log_critical_handler(void* /*user_data*/, const char* str) { printf("Critical: %s\n", str); }

/*************/
void log_warning_handler(void* /*user_data*/, const char* str) { printf("Warning: %s\n", str); }

/*************/
void log_message_handler(void* /*user_data*/, const char* str) { printf("Message: %s\n", str); }

/*************/
void log_info_handler(void* /*user_data*/, const char* str) { printf("Info: %s\n", str); }

/*************/
void log_debug_handler(void* user_data, const char* str) {
  bool show_debug = *((bool*)user_data);
  if (show_debug) printf("Debug: %s\n", str);
}

/*************/
// Any-data-writer
/*************/
void Writer_dealloc(pyshmdata_WriterObject* self) {
  Py_XDECREF(self->path);
  Py_XDECREF(self->datatype);
  Py_XDECREF(self->framesize);
  
  auto* state = PyEval_SaveThread();
  if (self->writer != nullptr) shmdata_delete_writer(self->writer);
  PyEval_RestoreThread(state);

  Py_TYPE(self)->tp_free((PyObject*)self);
}

/*************/
PyObject* Writer_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyshmdata_WriterObject* self;

  self = (pyshmdata_WriterObject*)type->tp_alloc(type, 0);
  if (self != nullptr) {
    self->path = PyUnicode_FromString("");
    if (self->path == nullptr) {
      Py_DECREF(self);
      return nullptr;
    }

    self->datatype = PyUnicode_FromString("");
    if (self->datatype == nullptr) {
      Py_DECREF(self);
      return nullptr;
    }

    self->framesize = PyLong_FromSize_t((size_t)4000000);
    if (self->framesize == nullptr) {
      Py_DECREF(self);
      return nullptr;
    }

    self->logger = shmdata_make_logger(&log_error_handler,
                                       &log_critical_handler,
                                       &log_warning_handler,
                                       &log_message_handler,
                                       &log_info_handler,
                                       &log_debug_handler,
                                       &self->show_debug_messages);
   
  }

  return (PyObject*)self;
}

/*************/
PyDoc_STRVAR(Writer_doc__,
             "PyShmdata Writer object\n"
             "\n"
             "Args:\n"
             "   path (str): Path to the shmdata\n"
             "   datatype (str): Caps of the data to send\n"
             "   framesize (int): Estimated frame size in bytes (can vary through time)\n"
             "   debug (bool): If true, activates debug mode\n"
             "\n"
             "Example use:\n"
             "   writer = pyshmdata.Writer(path=\"/path/to/shm\", datatype=\"video/x-raw, "
             "format=(string)RGBA, width=(int)512, height=(int)512\")\n"
             "   writer.push(some_buffer)");

int Writer_init(pyshmdata_WriterObject* self, PyObject* args, PyObject* kwds) {
  PyObject* path = nullptr;
  PyObject* datatype = nullptr;
  PyObject* framesize = nullptr;
  PyObject* showDebug = nullptr;
  PyObject* tmp = nullptr;

  static char* kwlist[] = {
      (char*)"path", (char*)"datatype", (char*)"framesize", (char*)"debug", nullptr};
  if (!PyArg_ParseTupleAndKeywords(
          args, kwds, "O|OOO", kwlist, &path, &datatype, &framesize, &showDebug))
    return -1;

  PyEval_InitThreads();

  tmp = self->path;
  Py_INCREF(path);
  self->path = path;
  Py_XDECREF(tmp);

  if (datatype) {
    tmp = self->datatype;
    Py_INCREF(datatype);
    self->datatype = datatype;
    Py_XDECREF(tmp);
  }

  if (framesize) {
    tmp = self->framesize;
    Py_INCREF(framesize);
    self->framesize = framesize;
    Py_XDECREF(tmp);
  }

  if (showDebug) {
    if (PyBool_Check(showDebug))
      self->show_debug_messages = true;
    else
      self->show_debug_messages = false;
  }

  string strPath(PyUnicode_AsUTF8(self->path));
  string strDatatype(PyUnicode_AsUTF8(self->datatype));
  size_t size = PyLong_AsSize_t(self->framesize);

  auto* state = PyEval_SaveThread();
  self->writer = shmdata_make_writer(
      strPath.c_str(), size, strDatatype.c_str(), nullptr, nullptr, nullptr, self->logger);
  PyEval_RestoreThread(state);
  if (!self->writer) {
    return -1;
  }

  return 0;
}

/*************/
PyDoc_STRVAR(Writer_push_doc__,
             "Push data through shmdata\n"
             "\n"
             "Args:\n"
             "   buffer (bytearray): Buffer object to push\n"
             "\n"
             "Returns:\n"
             "   True if all went well, false otherwise\n");

PyObject* Writer_push(pyshmdata_WriterObject* self, PyObject* args, PyObject* kwds) {
  PyObject* buffer = nullptr;

  static char* kwlist[] = {(char*)"buffer", nullptr};
  if (!PyArg_ParseTupleAndKeywords(args, kwds, "O", kwlist, &buffer)) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  if (!PyObject_TypeCheck(buffer, &PyByteArray_Type)) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  if (!self->writer) {
    Py_INCREF(Py_False);
    return Py_False;
  }

  if (self->writer) {
    Py_INCREF(buffer);
    void* buf = PyByteArray_AsString(buffer);
    size_t bufsize = PyByteArray_Size(buffer);
    auto* state = PyEval_SaveThread();
    shmdata_copy_to_shm(self->writer, buf, bufsize);
    PyEval_RestoreThread(state);
    Py_XDECREF(buffer);
  }

  Py_INCREF(Py_True);
  return Py_True;
}

/*************/
PyMemberDef Writer_members[] = {{(char*)"path",
                                 T_OBJECT_EX,
                                 offsetof(pyshmdata_WriterObject, path),
                                 0,
                                 (char*)"Path to the shmdata output"},
                                {(char*)"datatype",
                                 T_OBJECT_EX,
                                 offsetof(pyshmdata_WriterObject, datatype),
                                 0,
                                 (char*)"Type of the data sent"},
                                {(char*)"framesize",
                                 T_OBJECT_EX,
                                 offsetof(pyshmdata_WriterObject, framesize),
                                 0,
                                 (char*)"Size of the shared memory"},
                                {nullptr}};

PyMethodDef Writer_methods[] = {
    {(char*)"push", (PyCFunction)Writer_push, METH_VARARGS | METH_KEYWORDS, Writer_push_doc__},
    {nullptr}};

PyTypeObject pyshmdata_WriterType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyshmdata.Writer", /* tp_name */
    sizeof(pyshmdata_WriterObject),                           /* tp_basicsize */
    0,                                                        /* tp_itemsize */
    (destructor)Writer_dealloc,                               /* tp_dealloc */
    0,                                                        /* tp_print */
    0,                                                        /* tp_getattr */
    0,                                                        /* tp_setattr */
    0,                                                        /* tp_reserved */
    0,                                                        /* tp_repr */
    0,                                                        /* tp_as_number */
    0,                                                        /* tp_as_sequence */
    0,                                                        /* tp_as_mapping */
    0,                                                        /* tp_hash  */
    0,                                                        /* tp_call */
    0,                                                        /* tp_str */
    0,                                                        /* tp_getattro */
    0,                                                        /* tp_setattro */
    0,                                                        /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,                 /* tp_flags */
    Writer_doc__,                                             /* tp_doc */
    0,                                                        /* tp_traverse */
    0,                                                        /* tp_clear */
    0,                                                        /* tp_richcompare */
    0,                                                        /* tp_weaklistoffset */
    0,                                                        /* tp_iter */
    0,                                                        /* tp_iternext */
    Writer_methods,                                           /* tp_methods */
    Writer_members,                                           /* tp_members */
    0,                                                        /* tp_getset */
    0,                                                        /* tp_base */
    0,                                                        /* tp_dict */
    0,                                                        /* tp_descr_get */
    0,                                                        /* tp_descr_set */
    0,                                                        /* tp_dictoffset */
    (initproc)Writer_init,                                    /* tp_init */
    0,                                                        /* tp_alloc */
    Writer_new                                                /* tp_new */
};

/*************/
// Any-data-reader
void Reader_dealloc(pyshmdata_ReaderObject* self) {
  auto* state = PyEval_SaveThread();
  if (self->reader) shmdata_delete_follower(self->reader);
  PyEval_RestoreThread(state);
  
  Py_XDECREF(self->path);
  Py_XDECREF(self->datatype);
  if (nullptr != self->callback_user_data) Py_XDECREF(self->callback_user_data);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

/*************/
PyObject* Reader_new(PyTypeObject* type, PyObject* /*args*/, PyObject* /*kwds*/) {
  pyshmdata_ReaderObject* self;

  self = (pyshmdata_ReaderObject*)type->tp_alloc(type, 0);
  if (self != nullptr) {
    self->path = PyUnicode_FromString("");
    if (self->path == nullptr) {
      Py_DECREF(self);
      return nullptr;
    }

    self->datatype = PyUnicode_FromString("");
    if (self->datatype == nullptr) {
      Py_DECREF(self);
      return nullptr;
    }

    self->logger = shmdata_make_logger(&log_error_handler,
                                       &log_critical_handler,
                                       &log_warning_handler,
                                       &log_message_handler,
                                       &log_info_handler,
                                       &log_debug_handler,
                                       &self->show_debug_messages);
  }

  return (PyObject*)self;
}

/*************/
PyDoc_STRVAR(Reader_doc__,
             "PyShmdata Reader object\n"
             "\n"
             "Args:\n"
             "   path (str): Path to the shmdata\n"
             "   callback (function): Function to be called when a new buffer is received\n"
             "   user_data (object): Object to set as a parameter for the callback\n"
             "   drop_frames (bool): If true, frames are drop if processing is too slow\n"
             "   debug (bool): If true, activates debug mode\n"
             "\n"
             "Callback signature:\n:"
             "   def callback(user_data, buffer, datatype, parsed_datatype)\n"
             "\n"
             "Example use:\n"
             "   reader = pyshmdata.Reader(path=\"/path/to/shm\"\n)"
             "   buf = reader.pull()");

int Reader_init(pyshmdata_ReaderObject* self, PyObject* args, PyObject* kwds) {
  PyObject* path = nullptr;
  PyObject* pyFunc = nullptr;
  PyObject* pyUserData = nullptr;
  PyObject* pyDropFrames = nullptr;
  PyObject* showDebug = nullptr;
  PyObject* tmp = nullptr;

  static char* kwlist[] = {(char*)"path",
                           (char*)"callback",
                           (char*)"user_data",
                           (char*)"drop_frames",
                           (char*)"debug",
                           nullptr};
  if (!PyArg_ParseTupleAndKeywords(
          args, kwds, "O|OOOO", kwlist, &path, &pyFunc, &pyUserData, &pyDropFrames, &showDebug))
    return -1;

  PyEval_InitThreads();

  if (path) {
    tmp = self->path;
    Py_INCREF(path);
    self->path = path;
    Py_XDECREF(tmp);
  }

  if (PyCallable_Check(pyFunc)) {
    tmp = self->callback;
    Py_INCREF(pyFunc);
    self->callback = pyFunc;
    Py_XDECREF(tmp);
  }

  if (pyUserData) {
    tmp = self->callback_user_data;
    Py_INCREF(pyUserData);
    self->callback_user_data = pyUserData;
    Py_XDECREF(tmp);
  }

  if (pyDropFrames) {
    self->drop_frames = PyObject_IsTrue(pyDropFrames);
  }

  if (showDebug) {
    if (PyBool_Check(showDebug))
      self->show_debug_messages = true;
    else
      self->show_debug_messages = false;
  }

  PyEval_InitThreads();
  auto* state = PyEval_SaveThread();
  self->reader = shmdata_make_follower(PyUnicode_AsUTF8(self->path),
                                       Reader_on_data_handler,
                                       Reader_on_connect_handler,
                                       Reader_on_disconnect,
                                       self,
                                       self->logger);
  PyEval_RestoreThread(state);
  if (!self->reader) return -1;

  return 0;
}

/*************/
PyDoc_STRVAR(
    Reader_pull_doc__,
    "Pull last data from shmdata\n"
    "\n"
    "reader.pull()\n"
    "\n"
    "Returns:\n"
    "   Return a buffer holding the data, or an empty buffer if no data has been received");
PyObject* Reader_pull(pyshmdata_ReaderObject* self) {
  if (self->lastBuffer != nullptr) {
    Py_INCREF(self->lastBuffer);
    return self->lastBuffer;
  } else
    return Py_BuildValue("");
}

/*************/
void Reader_on_data_handler(void* user_data, void* data, size_t data_size) {
  pyshmdata_ReaderObject* self = static_cast<pyshmdata_ReaderObject*>(user_data);
  if (user_data == nullptr || (self->drop_frames && !self->frame_mutex.try_lock())) return;

  bool has_gil = (1 == PyGILState_Check()) ? true : false;
  PyGILState_STATE gil;
  if (!has_gil) {
    gil = PyGILState_Ensure();
  }

  // Get the current buffer
  PyObject* buffer = PyByteArray_FromStringAndSize((char*)data, data_size);
  PyObject* tmp = nullptr;
  if (self->lastBuffer != nullptr) tmp = self->lastBuffer;
  self->lastBuffer = buffer;
  if (tmp != nullptr) Py_XDECREF(tmp);

  // Call the callback, if present
  if (self->callback != nullptr && self->lastBuffer != nullptr) {
    PyObject* arglist;
    if (self->callback_user_data == nullptr)
      arglist =
          Py_BuildValue("(OOOO)", Py_None, self->lastBuffer, self->datatype, self->parsed_datatype);
    else
      arglist = Py_BuildValue("(OOOO)",
                              self->callback_user_data,
                              self->lastBuffer,
                              self->datatype,
                              self->parsed_datatype);
    PyObject* pyobjresult = PyEval_CallObject(self->callback, arglist);
    PyObject* pyerr = PyErr_Occurred();
    if (pyerr != nullptr) PyErr_Print();
    Py_DECREF(arglist);
    Py_XDECREF(pyobjresult);
  }

  if (!has_gil) {
    PyGILState_Release(gil);
  }

  if (self->drop_frames) self->frame_mutex.unlock();
}

/*************/
void Reader_on_connect_handler(void* user_data, const char* type_descr) {
  pyshmdata_ReaderObject* self = static_cast<pyshmdata_ReaderObject*>(user_data);
      
  bool has_gil = (1 == PyGILState_Check()) ? true : false;
  PyGILState_STATE gil;
  if (!has_gil) {
    gil = PyGILState_Ensure();
  }
  PyObject* datatype = PyUnicode_FromString(type_descr);
  PyObject* tmp = self->datatype;
  self->datatype = datatype;
  self->parsed_datatype = Reader_parse_datatype(type_descr);
  Py_XDECREF(tmp);

  if (!has_gil) {
    PyGILState_Release(gil);
  }
}

/*************/
void Reader_on_disconnect(void* /*user_data*/) { return; }

/*************/
PyObject* Reader_parse_datatype(const char* type_descr) {
  auto datatype = string(type_descr);
  PyObject* dict = PyDict_New();

  auto space_pos = datatype.find(' ');
  while (space_pos != string::npos) {
    datatype.replace(space_pos, 1, "");
    space_pos = datatype.find(' ');
  }

  while (!datatype.empty()) {
    auto substr = string();
    auto comma_pos = datatype.find(',');
    if (comma_pos == string::npos) {
      substr = datatype;
      datatype = "";
    } else {
      substr = datatype.substr(0, comma_pos);
      datatype = datatype.substr(comma_pos + 1);
    }

    if (substr.find('=') == string::npos) {
      PyObject* dictkey = PyUnicode_FromString("type");
      PyObject* value = PyUnicode_FromString(substr.c_str());
      PyDict_SetItem(dict, dictkey, value);
      Py_DECREF(dictkey);
      Py_DECREF(value);
    } else {
      PyObject* value = nullptr;

      auto equal_pos = substr.find('=');
      auto val = substr.substr(equal_pos + 1);
      auto key = substr[0] == ' ' ? substr.substr(1, equal_pos) : substr.substr(0, equal_pos);

      auto open_parenthesis = val.find('(');
      auto close_parenthesis = val.find(')');
      if (open_parenthesis != string::npos && close_parenthesis != string::npos) {
        if (val.substr(open_parenthesis + 1, close_parenthesis - 1) == "int") {
          val.replace(open_parenthesis, close_parenthesis - open_parenthesis + 1, "");
          value = Py_BuildValue("I", stoi(val));
        } else if (val.substr(open_parenthesis + 1, close_parenthesis - 1) == "fraction") {
          val.replace(open_parenthesis, close_parenthesis - open_parenthesis + 1, "");
          auto denominator = stof(val.substr(0, val.find('/')));
          auto numerator = stof(val.substr(val.find('/') + 1));
          auto float_value = denominator / numerator;
          value = Py_BuildValue("f", float_value);
        } else {
          val.replace(open_parenthesis, close_parenthesis - open_parenthesis + 1, "");
          value = PyUnicode_FromString(val.c_str());
        }
      } else {
        value = PyUnicode_FromString(val.c_str());
      }

      PyObject* dictkey = PyUnicode_FromString(key.c_str());
      PyDict_SetItem(dict, dictkey, value);
      Py_DECREF(dictkey);
      Py_DECREF(value);
    }
  }

  return dict;
}

/*************/
PyMemberDef Reader_members[] = {{(char*)"path",
                                 T_OBJECT_EX,
                                 offsetof(pyshmdata_ReaderObject, path),
                                 0,
                                 (char*)"Path to the shmdata input"},
                                {(char*)"datatype",
                                 T_OBJECT_EX,
                                 offsetof(pyshmdata_ReaderObject, datatype),
                                 0,
                                 (char*)"Type of the data received"},
                                {(char*)"parsed_datatype",
                                 T_OBJECT_EX,
                                 offsetof(pyshmdata_ReaderObject, parsed_datatype),
                                 0,
                                 (char*)"Parsed data type"},
                                {nullptr}};

PyMethodDef Reader_methods[] = {
    {(char*)"pull", (PyCFunction)Reader_pull, METH_VARARGS, Reader_pull_doc__}, {nullptr}};

PyTypeObject pyshmdata_ReaderType = {
    PyVarObject_HEAD_INIT(nullptr, 0)(char*) "pyshmdata.Reader", /* tp_name */
    sizeof(pyshmdata_ReaderObject),                           /* tp_basicsize */
    0,                                                        /* tp_itemsize */
    (destructor)Reader_dealloc,                               /* tp_dealloc */
    0,                                                        /* tp_print */
    0,                                                        /* tp_getattr */
    0,                                                        /* tp_setattr */
    0,                                                        /* tp_reserved */
    0,                                                        /* tp_repr */
    0,                                                        /* tp_as_number */
    0,                                                        /* tp_as_sequence */
    0,                                                        /* tp_as_mapping */
    0,                                                        /* tp_hash  */
    0,                                                        /* tp_call */
    0,                                                        /* tp_str */
    0,                                                        /* tp_getattro */
    0,                                                        /* tp_setattro */
    0,                                                        /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,                 /* tp_flags */
    Reader_doc__,                                             /* tp_doc */
    0,                                                        /* tp_traverse */
    0,                                                        /* tp_clear */
    0,                                                        /* tp_richcompare */
    0,                                                        /* tp_weaklistoffset */
    0,                                                        /* tp_iter */
    0,                                                        /* tp_iternext */
    Reader_methods,                                           /* tp_methods */
    Reader_members,                                           /* tp_members */
    0,                                                        /* tp_getset */
    0,                                                        /* tp_base */
    0,                                                        /* tp_dict */
    0,                                                        /* tp_descr_get */
    0,                                                        /* tp_descr_set */
    0,                                                        /* tp_dictoffset */
    (initproc)Reader_init,                                    /* tp_init */
    0,                                                        /* tp_alloc */
    Reader_new                                                /* tp_new */
};

/*************/
PyMODINIT_FUNC PyInit_pyshmdata(void) {
  PyObject* m;

  if (PyType_Ready(&pyshmdata_WriterType) < 0) return nullptr;
  if (PyType_Ready(&pyshmdata_ReaderType) < 0) return nullptr;

  m = PyModule_Create(&pyshmdatamodule);
  if (m == nullptr) return nullptr;

  Py_INCREF(&pyshmdata_WriterType);
  Py_INCREF(&pyshmdata_ReaderType);
  PyModule_AddObject(m, "Writer", (PyObject*)&pyshmdata_WriterType);
  PyModule_AddObject(m, "Reader", (PyObject*)&pyshmdata_ReaderType);

  return m;
}
