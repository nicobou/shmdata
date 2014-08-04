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

#include "any-data-reader.h"
#include "any-data-writer.h"

#include <Python.h>
#include "structmember.h"

#ifdef __cplusplush
extern "C"
{
#endif

/*************/
// Any-data-writer
typedef struct {
    PyObject_HEAD
    PyObject* path {NULL};
    PyObject* datatype {NULL};
    shmdata_any_writer_t* writer {NULL};
} pyshmdata_WriterObject;

static void Writer_dealloc(pyshmdata_WriterObject* self);
static PyObject* Writer_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
static int Writer_init(pyshmdata_WriterObject* self, PyObject* args, PyObject* kwds);
static PyObject* Writer_push(pyshmdata_WriterObject* self, PyObject* args);
static void Writer_freeObject(void* user_data);

static PyMemberDef Writer_members[] = {
    {(char*)"path", T_OBJECT_EX, offsetof(pyshmdata_WriterObject, path), 0, (char*)"Path to the shmdata output"},
    {(char*)"datatype", T_OBJECT_EX, offsetof(pyshmdata_WriterObject, datatype), 0, (char*)"Type of the data sent"},
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
    shmdata_any_reader_t* reader;
} pyshmdata_readerObject;

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

#ifdef __cplusplush
}
#endif

#endif
