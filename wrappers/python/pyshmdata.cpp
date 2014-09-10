/*
 * Copyright (C) 2014 Emmanuel Durand (https://emmanueldurand.net)
 *
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

#include <iostream>
#include <string>

using namespace std;

/*************/
// Any-data-writer
PyMODINIT_FUNC
PyInit_pyshmdata(void)
{
    PyObject* m;

    if (PyType_Ready(&pyshmdata_WriterType) < 0)
        return NULL;
    if (PyType_Ready(&pyshmdata_ReaderType) < 0)
        return NULL;

    m = PyModule_Create(&pyshmdatamodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&pyshmdata_WriterType);
    Py_INCREF(&pyshmdata_ReaderType);
    PyModule_AddObject(m, "Writer", (PyObject*)&pyshmdata_WriterType);
    PyModule_AddObject(m, "Reader", (PyObject*)&pyshmdata_ReaderType);

    return m;
}

/*************/
void
Writer_dealloc(pyshmdata_WriterObject* self)
{
    Py_XDECREF(self->path);
    Py_XDECREF(self->datatype);
    if (self->writer != NULL)
        shmdata_any_writer_close(self->writer);

    Py_TYPE(self)->tp_free((PyObject*)self);
}

/*************/
PyObject*
Writer_new(PyTypeObject* type, PyObject* args, PyObject* kwds)
{
    pyshmdata_WriterObject* self;

    self = (pyshmdata_WriterObject*)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->path = PyUnicode_FromString("");
        if (self->path == NULL) {
            Py_DECREF(self);
            return NULL;
        }

        self->datatype = PyUnicode_FromString("");
        if (self->datatype == NULL) {
            Py_DECREF(self);
            return NULL;
        }
    }

    return (PyObject*)self;
}

/*************/
int
Writer_init(pyshmdata_WriterObject* self, PyObject* args, PyObject* kwds)
{
    PyObject *path = NULL;
    PyObject *datatype = NULL;
    PyObject *tmp = NULL;

    static char *kwlist[] = {(char*)"path", (char*)"datatype", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OO", kwlist,
                                     &path, &datatype))
        return -1;

    if (path) {
        tmp = self->path;
        Py_INCREF(path);
        self->path = path;
        Py_XDECREF(tmp);
    }

    if (datatype) {
        tmp = self->datatype;
        Py_INCREF(datatype);
        self->datatype = datatype;
        Py_XDECREF(tmp);
    }

    return 0;
}

/*************/
PyObject*
Writer_push(pyshmdata_WriterObject* self, PyObject* args)
{
    PyObject* buffer = NULL;
    unsigned long long timestamp = 0;

    if (!PyArg_ParseTuple(args, "O|K", &buffer, &timestamp)) {
        return NULL;
    }

    if (!PyObject_TypeCheck(buffer, &PyByteArray_Type)) {
        return NULL;
    }

    if (self->writer == NULL) {
        string path(PyUnicode_AsUTF8(self->path));
        string datatype(PyUnicode_AsUTF8(self->datatype));

        self->writer = shmdata_any_writer_init();
        if (self->writer == NULL) {
            return NULL;
        }

        shmdata_any_writer_set_data_type(self->writer, datatype.c_str());
        if (!shmdata_any_writer_set_path(self->writer, path.c_str())) {
            return NULL;
        }

        shmdata_any_writer_start(self->writer);
    }

    if (self->writer != NULL) {
        Py_INCREF(buffer);
        shmdata_any_writer_push_data(self->writer, (void*)PyByteArray_AsString(buffer), PyByteArray_Size(buffer), timestamp, Writer_freeObject, buffer);
    }

    return Py_BuildValue("i", 1);
}

/*************/
void
Writer_freeObject(void* user_data)
{
    PyObject* buffer = (PyObject*)user_data;
    Py_DECREF(buffer);
}

/*************/
// Any-data-reader
void
Reader_dealloc(pyshmdata_ReaderObject* self)
{
    lock_guard<mutex> lock(self->reader_mutex);

    Py_XDECREF(self->path);
    Py_XDECREF(self->datatype);
    if (self->reader != NULL)
        shmdata_any_reader_close(self->reader);

    Py_TYPE(self)->tp_free((PyObject*)self);
}

/*************/
PyObject*
Reader_new(PyTypeObject* type, PyObject* args, PyObject* kwds)
{
    pyshmdata_ReaderObject* self;

    self = (pyshmdata_ReaderObject*)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->path = PyUnicode_FromString("");
        if (self->path == NULL) {
            Py_DECREF(self);
            return NULL;
        }

        self->datatype = PyUnicode_FromString("");
        if (self->datatype == NULL) {
            Py_DECREF(self);
            return NULL;
        }
    }

    return (PyObject*)self;
}

/*************/
int
Reader_init(pyshmdata_ReaderObject* self, PyObject* args, PyObject* kwds)
{
    PyObject *path = NULL;
    PyObject *pyFunc = NULL;
    PyObject *pyUserData = NULL;
    PyObject *tmp = NULL;

    static char *kwlist[] = {(char*)"path", (char*)"callback", (char*)"user_data", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|OO", kwlist, &path, &pyFunc, &pyUserData))
        return -1;

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

    self->reader = shmdata_any_reader_init();
    if (self->reader == NULL)
        return NULL;
    shmdata_any_reader_set_on_data_handler(self->reader, Reader_on_data_handler, self);
    shmdata_any_reader_start(self->reader, PyUnicode_AsUTF8(self->path));

    return 0;
}

/*************/
PyObject*
Reader_pull(pyshmdata_ReaderObject* self)
{
    return self->lastBuffer;
}

/*************/
void
Reader_freeObject(void* user_data)
{
    PyObject* buffer = (PyObject*)user_data;
    Py_DECREF(buffer);
}

/*************/
void
Reader_on_data_handler(shmdata_any_reader_t* reader, void* shmbuf, void* data, int data_size, unsigned long long timestamp, const char* type, void* user_data)
{
    if (user_data == nullptr)
    {
        shmdata_any_reader_free(shmbuf);
        return;
    }

    pyshmdata_ReaderObject* self = static_cast<pyshmdata_ReaderObject*>(user_data);

    lock_guard<mutex> lock(self->reader_mutex);

    // Get the current type
    PyObject *datatype = NULL;
    PyObject *tmp = NULL;

    datatype = PyUnicode_FromString(type);
    tmp = self->datatype;
    self->datatype = datatype;
    Py_XDECREF(tmp);

    // Get the current buffer
    PyObject *buffer = NULL;

    buffer = PyByteArray_FromStringAndSize((char*)data, data_size);
    tmp = NULL;
    if (self->lastBuffer != NULL)
        tmp = self->lastBuffer;
    self->lastBuffer = buffer;
    if (tmp != NULL)
        Py_XDECREF(tmp);

    // Call the callback, if present
    if (self->callback != NULL && self->lastBuffer != NULL)
    {
        PyGILState_STATE gil = PyGILState_Ensure();
        PyObject *arglist;
        if (self->callback_user_data == NULL)
            arglist = Py_BuildValue("(OOO)", Py_None, self->lastBuffer, self->datatype);
        else
            arglist = Py_BuildValue("(OOO)", self->callback_user_data, self->lastBuffer, self->datatype);
        PyObject *pyobjresult = PyEval_CallObject(self->callback, arglist);
        PyObject *pyerr = PyErr_Occurred();
        if (pyerr != NULL)
            PyErr_Print();
        Py_DECREF(arglist);
        Py_XDECREF(pyobjresult);
        PyGILState_Release(gil);
    }
    
    shmdata_any_reader_free(shmbuf);
}
