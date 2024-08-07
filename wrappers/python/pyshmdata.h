/*
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
#include <structmember.h>
#include <memory>

#include "shmdata/cfollower.h"
#include "shmdata/clogger.h"
#include "shmdata/cwriter.h"

#ifdef __cplusplus
extern "C" {
#endif

/*************/
// Any-data-writer
typedef struct pyshmdata_WriterObject_t {
  PyObject_HEAD PyObject* path{NULL};
  PyObject* datatype{NULL};
  PyObject* framesize{NULL};
  ShmdataLogger logger{NULL};
  ShmdataWriter writer{NULL};
  bool show_debug_messages{false};
} pyshmdata_WriterObject;

void Writer_dealloc(pyshmdata_WriterObject* self);
PyObject* Writer_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
int Writer_init(pyshmdata_WriterObject* self, PyObject* args, PyObject* kwds);
PyObject* Writer_push(pyshmdata_WriterObject* self, PyObject* args, PyObject* kwds);
// static void Writer_freeObject(void* user_data);

/*************/
// Any-data-reader
typedef struct pyshmdata_ReaderObject_t {
  PyObject_HEAD PyObject* path{NULL};
  PyObject* datatype{NULL};
  PyObject* parsed_datatype{NULL};
  PyObject* lastBuffer{NULL};
  PyObject* callback{NULL};
  PyObject* callback_user_data{NULL};
  // shmdata_any_reader_t* reader;
  ShmdataLogger logger{NULL};
  ShmdataFollower reader{NULL};
  std::mutex frame_mutex;
  bool drop_frames{false};
  bool show_debug_messages{false};
} pyshmdata_ReaderObject;

void Reader_dealloc(pyshmdata_ReaderObject* self);
PyObject* Reader_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
int Reader_init(pyshmdata_ReaderObject* self, PyObject* args, PyObject* kwds);
PyObject* Reader_pull(pyshmdata_ReaderObject* self);
void Reader_on_data_handler(void* user_data, void* data, size_t data_size);
void Reader_on_connect_handler(void* user_data, const char* type_descr);
void Reader_on_disconnect(void* user_data);
PyObject* Reader_parse_datatype(const char* type_descr);

/*************/
// Module
static PyModuleDef pyshmdatamodule = {
    PyModuleDef_HEAD_INIT, "pyshmdata", "Shmdata module", -1, NULL, NULL, NULL, NULL, NULL};

PyMODINIT_FUNC PyInit_pyshmdata(void);

#ifdef __cplusplus
}
#endif

#endif
