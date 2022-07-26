#include "./pylogger.hpp"

PyObject* pyLogger::tp_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
  pyLoggerObject* self;
  self = (pyLoggerObject*)type->tp_alloc(type, 0);
  return reinterpret_cast<PyObject*>(self);
}

int pyLogger::tp_init(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  self->logger = spdlog::get("switcher");
  return 0;
}

void pyLogger::tp_dealloc(pyLoggerObject* self) {
  std::shared_ptr<spdlog::logger> empty;
  self->logger.swap(empty);
  Py_TYPE(self)->tp_free((PyObject*)self);
}

PyObject* pyLogger::trace(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->trace(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::debug(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->debug(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::info(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->info(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::warn(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->warn(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::error(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->error(msg);
  Py_RETURN_TRUE;
}

PyObject* pyLogger::critical(pyLoggerObject* self, PyObject* args, PyObject* kwds) {
  const char* msg = nullptr;
  if (!PyArg_ParseTuple(args, "s", &msg)) Py_RETURN_FALSE;
  self->logger->critical(msg);
  Py_RETURN_TRUE;
}

PyMethodDef pyLogger::methods[] = {
    {"trace",
     (PyCFunction)pyLogger::trace,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `TRACE` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"debug",
     (PyCFunction)pyLogger::debug,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `DEBUG` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"info",
     (PyCFunction)pyLogger::info,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `INFO` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"warn",
     (PyCFunction)pyLogger::warn,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `WARN` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"error",
     (PyCFunction)pyLogger::error,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `ERROR` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {"critical",
     (PyCFunction)pyLogger::critical,
     METH_VARARGS,
     PyDoc_STR("Logs a message with level `CRITICAL` on this logger.\n"
               "----------\n"
               "msg: str\n"
               "\tA formatted string that should be logged")},
    {nullptr}  // sentinel
};

PyTypeObject pyLogger::pyType = {
    PyVarObject_HEAD_INIT(nullptr, 0).tp_name = "pyLogger",
    .tp_basicsize = sizeof(pyLoggerObject),
    .tp_dealloc = (destructor)tp_dealloc,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = "The `Logger` for Switcher.",
    .tp_methods = methods,
    .tp_init = (initproc)tp_init,
    .tp_new = (newfunc)tp_new,
};