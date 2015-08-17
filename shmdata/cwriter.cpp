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

namespace shmdata{
struct CWriter {
  CWriter(const char *path,
          size_t memsize,
          const char *type_descr,
          ShmdataLogger log,
          void (*on_client_connected)(void *user_data, int id),
          void (*on_client_disconnected)(void *user_data, int id),
          void *user_data) :
      on_client_connected_(on_client_connected),
      on_client_disconnected_(on_client_disconnected),
      user_data_(user_data),
      writer_(path,
              memsize,
              type_descr,
              static_cast<AbstractLogger *>(log),
              [&](int id) {
                if(nullptr != on_client_connected_)
                  on_client_connected_(user_data_, id);
              },
              [&](int id) {
                if(nullptr != on_client_disconnected_)
                  on_client_disconnected_(user_data_, id);
              }){
  }
  void (*on_client_connected_)(void *, int);
  void (*on_client_disconnected_)(void *, int);
  void *user_data_;
  Writer writer_;  
};

}  // namespace shmdata

using namespace shmdata;
ShmdataWriter shmdata_make_writer(const char *path,
                                  size_t memsize,
                                  const char *type_descr,
                                  void (*on_client_connected)(void *user_data, int id),
                                  void (*on_client_disconnected)(void *user_data, int id),
                                  void *user_data,
                                  ShmdataLogger log){
  CWriter *res = new CWriter(path,
                             memsize,
                             type_descr,
                             static_cast<AbstractLogger *>(log),
                             on_client_connected,
                             on_client_disconnected,
                             user_data);
  if (res->writer_)
    return static_cast<void *>(res);
  delete res;
  return nullptr;
}

void shmdata_delete_writer(ShmdataWriter writer){
  delete static_cast<CWriter *>(writer);
}

int shmdata_copy_to_shm(ShmdataWriter writer, void *data, size_t size){
  return static_cast<CWriter *>(writer)->writer_.copy_to_shm(data, size);
}

ShmdataWriterAccess shmdata_get_one_write_access(ShmdataWriter writer){
  return
      static_cast<void *>(static_cast<CWriter *>(writer)->writer_.
                          get_one_write_access_ptr());
}

void *shmdata_get_mem(ShmdataWriterAccess access){
  return static_cast<OneWriteAccess *>(access)->get_mem();
}

short shmdata_notify_clients(ShmdataWriterAccess access, size_t size){
  return static_cast<OneWriteAccess *>(access)->notify_clients(size);
}

void shmdata_release_one_write_access(ShmdataWriterAccess access){
  delete static_cast<OneWriteAccess *>(access);
}
