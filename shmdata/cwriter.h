/*
 * Copyright (C) 2015 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#ifndef _SHMDATA_C_WRITER_H_
#define _SHMDATA_C_WRITER_H_

#include <stdlib.h>
#include "./clogger.h"

typedef void * ShmdataWriter;
typedef void * ShmdataWriterAccess;
extern "C" ShmdataWriter shmdata_make_writer(const char * path,
                                             size_t memsize,
                                             const char *type_descr,
                                             ShmdataCLogger *log);
extern "C" void shmdata_delete_writer(ShmdataWriter writer);
extern "C" int shmdata_copy_to_shm(ShmdataWriter writer,
                                   void *data,
                                   size_t size);

extern "C" int shmdata_copy_to_shm(ShmdataWriter writer,
                                   void *data,
                                   size_t size);
extern "C" ShmdataWriterAccess shmdata_get_one_write_access(ShmdataWriter writer,
                                                            size_t size);
extern "C" void shmdata_release_one_write_access(ShmdataWriterAccess access);

#endif
