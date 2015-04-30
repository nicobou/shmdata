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

#include "./cwriter.h"
#include "./writer.hpp"

using namespace shmdata;

ShmdataWriter shmdata_make_writer(const char *path,
                                  size_t memsize,
                                  const char *type_descr,
                                  ShmdataLogger log){
  Writer *res = new Writer(path,
                           memsize,
                           type_descr,
                           static_cast<AbstractLogger *>(log));
  if (*res)
    return static_cast<void *>(res);
  delete res;
  return nullptr;
}

void shmdata_delete_writer(ShmdataWriter writer){
  delete static_cast<Writer *>(writer);
}

int shmdata_copy_to_shm(ShmdataWriter writer, void *data, size_t size){
  return static_cast<Writer *>(writer)->copy_to_shm(data, size);
}

ShmdataWriterAccess shmdata_get_one_write_access(ShmdataWriter writer,
                                                 size_t size){
  return
      static_cast<void *>(static_cast<Writer *>(writer)->get_one_write_access_ptr(size));
}

void *shmdata_get_mem(ShmdataWriterAccess access){
  return static_cast<OneWriteAccess *>(access)->get_mem();
}

short shmdata_notify_clients(ShmdataWriterAccess access){
  return static_cast<OneWriteAccess *>(access)->notify_clients();
}

void shmdata_release_one_write_access(ShmdataWriterAccess access){
  delete static_cast<OneWriteAccess *>(access);
}
