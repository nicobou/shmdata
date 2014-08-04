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

/*************/
PyMODINIT_FUNC
PyInit_pyshmdata(void)
{
    PyObject* m;

    pyshmdata_WriterType.tp_new = PyType_GenericNew;
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
static PyObject*
shmdata_writer_init(PyObject* self, PyObject* args)
{
    const char* filename;

    if (!PyArg_ParseTuple(args, "s", &filename))
        return NULL;
}
