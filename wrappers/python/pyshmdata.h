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

#ifndef __SHMDATA_PYSHMDATA_H__
#define __SHMDATA_PYSHMDATA_H__

#include <mutex>

#include <Python.h>
#include <memory>
#include <structmember.h>

#include "shmdata/clogger.h"
#include "shmdata/cwriter.h"
#include "shmdata/cfollower.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*************/
// Any-data-writer
typedef struct {
    PyObject_HEAD
    PyObject* path {NULL};
    PyObject* datatype {NULL};
    PyObject* framesize {NULL};
    ShmdataLogger logger {NULL};
    ShmdataWriter writer {NULL};
    bool show_debug_messages {false};
} pyshmdata_WriterObject;

static void Writer_dealloc(pyshmdata_WriterObject* self);
static PyObject* Writer_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
static int Writer_init(pyshmdata_WriterObject* self, PyObject* args, PyObject* kwds);
static PyObject* Writer_push(pyshmdata_WriterObject* self, PyObject* args);
//static void Writer_freeObject(void* user_data);

static PyMemberDef Writer_members[] = {
    {(char*)"path", T_OBJECT_EX, offsetof(pyshmdata_WriterObject, path), 0, (char*)"Path to the shmdata output"},
    {(char*)"datatype", T_OBJECT_EX, offsetof(pyshmdata_WriterObject, datatype), 0, (char*)"Type of the data sent"},
    {(char*)"framesize", T_OBJECT_EX, offsetof(pyshmdata_WriterObject, framesize), 0, (char*)"Size of the shared memory"},
    {NULL}
};

static PyMethodDef Writer_methods[] = {
    {(char*)"push", (PyCFunction)Writer_push, METH_VARARGS, (char*)"Push data through shmdata"},
    {NULL}
};

static PyTypeObject pyshmdata_WriterType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    (char*)"pyshmdata.Writer",          /* tp_name */
    sizeof(pyshmdata_WriterObject),     /* tp_basicsize */
    0,                                  /* tp_itemsize */
    (destructor)Writer_dealloc,         /* tp_dealloc */
    0,                                  /* tp_print */
    0,                                  /* tp_getattr */
    0,                                  /* tp_setattr */
    0,                                  /* tp_reserved */
    0,                                  /* tp_repr */
    0,                                  /* tp_as_number */
    0,                                  /* tp_as_sequence */
    0,                                  /* tp_as_mapping */
    0,                                  /* tp_hash  */
    0,                                  /* tp_call */
    0,                                  /* tp_str */
    0,                                  /* tp_getattro */
    0,                                  /* tp_setattro */
    0,                                  /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT
        | Py_TPFLAGS_BASETYPE,          /* tp_flags */
    (char*)"PyShmdata Writer Object",   /* tp_doc */
    0,                                  /* tp_traverse */
    0,                                  /* tp_clear */
    0,                                  /* tp_richcompare */
    0,                                  /* tp_weaklistoffset */
    0,                                  /* tp_iter */
    0,                                  /* tp_iternext */
    Writer_methods,                     /* tp_methods */
    Writer_members,                     /* tp_members */
    0,                                  /* tp_getset */
    0,                                  /* tp_base */
    0,                                  /* tp_dict */
    0,                                  /* tp_descr_get */
    0,                                  /* tp_descr_set */
    0,                                  /* tp_dictoffset */
    (initproc)Writer_init,              /* tp_init */
    0,                                  /* tp_alloc */
    Writer_new                          /* tp_new */
};

/*************/
// Any-data-reader
typedef struct {
    PyObject_HEAD
    PyObject* path {NULL};
    PyObject* datatype {NULL};
    PyObject* lastBuffer {NULL};
    PyObject* callback {NULL};
    PyObject* callback_user_data {NULL};
    //shmdata_any_reader_t* reader;
    ShmdataLogger logger {NULL};
    ShmdataFollower reader {NULL};
    std::mutex reader_mutex;
    std::mutex frame_mutex;
    bool drop_frames {false};
    bool show_debug_messages {false};
} pyshmdata_ReaderObject;

static void Reader_dealloc(pyshmdata_ReaderObject* self);
static PyObject* Reader_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
static int Reader_init(pyshmdata_ReaderObject* self, PyObject* args, PyObject* kwds);
static PyObject* Reader_pull(pyshmdata_ReaderObject* self);
static void Reader_on_data_handler(void *user_data, void *data, size_t data_size);
static void Reader_on_connect_handler(void *user_data, const char *type_descr);
static void Reader_on_disconnect(void *user_data);

static PyMemberDef Reader_members[] = {
    {(char*)"path", T_OBJECT_EX, offsetof(pyshmdata_ReaderObject, path), 0, (char*)"Path to the shmdata input"},
    {(char*)"datatype", T_OBJECT_EX, offsetof(pyshmdata_ReaderObject, datatype), 0, (char*)"Type of the data received"},
    {NULL}
};

static PyMethodDef Reader_methods[] = {
    {(char*)"pull", (PyCFunction)Reader_pull, METH_VARARGS, (char*)"Pull last data from shmdata"},
    {NULL}
};

static PyTypeObject pyshmdata_ReaderType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    (char*)"pyshmdata.Reader",          /* tp_name */
    sizeof(pyshmdata_ReaderObject),     /* tp_basicsize */
    0,                                  /* tp_itemsize */
    (destructor)Reader_dealloc,         /* tp_dealloc */
    0,                                  /* tp_print */
    0,                                  /* tp_getattr */
    0,                                  /* tp_setattr */
    0,                                  /* tp_reserved */
    0,                                  /* tp_repr */
    0,                                  /* tp_as_number */
    0,                                  /* tp_as_sequence */
    0,                                  /* tp_as_mapping */
    0,                                  /* tp_hash  */
    0,                                  /* tp_call */
    0,                                  /* tp_str */
    0,                                  /* tp_getattro */
    0,                                  /* tp_setattro */
    0,                                  /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT
        | Py_TPFLAGS_BASETYPE,          /* tp_flags */
    (char*)"PyShmdata Reader Object",   /* tp_doc */
    0,                                  /* tp_traverse */
    0,                                  /* tp_clear */
    0,                                  /* tp_richcompare */
    0,                                  /* tp_weaklistoffset */
    0,                                  /* tp_iter */
    0,                                  /* tp_iternext */
    Reader_methods,                     /* tp_methods */
    Reader_members,                     /* tp_members */
    0,                                  /* tp_getset */
    0,                                  /* tp_base */
    0,                                  /* tp_dict */
    0,                                  /* tp_descr_get */
    0,                                  /* tp_descr_set */
    0,                                  /* tp_dictoffset */
    (initproc)Reader_init,              /* tp_init */
    0,                                  /* tp_alloc */
    Reader_new                          /* tp_new */
};

/*************/
// Module
static PyModuleDef pyshmdatamodule = {
    PyModuleDef_HEAD_INIT,
    "pyshmdata",
    "Shmdata module",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit_pyshmdata(void);

#ifdef __cplusplus
}
#endif

#endif
