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

#ifdef __cplusplush
extern "C"
{
#endif

/*************/
// Any-data-writer
typedef struct {
    PyObject_HEAD
    shmdata_any_writer_t* writer;
} pyshmdata_WriterObject;

static PyTypeObject pyshmdata_WriterType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "pyshmdata.Writer",                 /* tp_name */
    sizeof(pyshmdata_WriterObject),     /* tp_basicsize */
    0,                                  /* tp_itemsize */
    0,                                  /* tp_dealloc */
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
    Py_TPFLAGS_DEFAULT,                 /* tp_flags */
    "PyShmdata Writer Object",          /* tp_doc */
};

static PyModuleDef pyshmdatamodule = {
    PyModuleDef_HEAD_INIT,
    "pyshmdata",
    "Shmdata module",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit_pyshmdata(void);

/*************/
// Any-data-reader
typedef struct {
    PyObject_HEAD
    shmdata_any_reader_t* reader;
} pyshmdata_readerObject;

/*************/
static PyObject* shmdata_writer_init(PyObject* self, PyObject* args);

#ifdef __cplusplush
}
#endif

#endif
