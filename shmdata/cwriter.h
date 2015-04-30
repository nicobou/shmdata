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

#ifdef __cplusplus
extern "C" {
#endif
  
  typedef void * ShmdataWriter;
  typedef void * ShmdataWriterAccess;
  
  ShmdataWriter shmdata_make_writer(const char *path,
                                  size_t memsize,
                                  const char *type_descr,
                                  void (*on_client_connected)(void *user_data, int id),
                                  void (*on_client_disconnected)(void *user_data, int id),
                                  void *user_data,
                                  ShmdataLogger log);
  void shmdata_delete_writer(ShmdataWriter writer);

  // write copying data
  int shmdata_copy_to_shm(ShmdataWriter writer,
                          void *data,
                          size_t size);

  // or get write lock and notify clients when they can try locking for reading 
  ShmdataWriterAccess shmdata_get_one_write_access(ShmdataWriter writer);
  void *shmdata_get_mem(ShmdataWriterAccess access);
  short shmdata_notify_clients(ShmdataWriterAccess access, size_t size);
  void shmdata_release_one_write_access(ShmdataWriterAccess access);

#ifdef __cplusplus
}
#endif

#endif
