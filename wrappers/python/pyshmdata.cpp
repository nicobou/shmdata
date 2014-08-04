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

    m = PyModule_Create(&pyshmdatamodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&pyshmdata_WriterType);
    PyModule_AddObject(m, "Writer", (PyObject*)&pyshmdata_WriterType);
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
